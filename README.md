# LOS UAV Interception

这是从 `mappo_uav_project` 中独立出来的纯 LOS（Line of Sight，视线）无人机拦截项目。它不包含 PPO/MAPPO、Actor、Critic 或模型权重，只保留可直接部署和测试的解析导引、受限点质量动力学、传感器噪声、单机/三机仿真与命中函数 F 诊断。

## 1. 适用范围

- 输入：本机 NED 速度、靶机相对本机的 NED 位置；
- 可选输入：雷达、视觉跟踪器或状态估计器给出的 LOS 角速度；
- 输出：本机在 NED 坐标系下的三轴加速度指令，单位 `m/s²`；
- 不需要：靶机运动模式、未来航点、靶机控制指令或靶机真实速度；
- 仿真成功判定：连续轨迹最近距离小于 `0.6 m`；
- 命中函数 F：只记录诊断指标，不决定是否成功。

这是导航与算法验证代码，不是完整飞控。实机使用时仍需姿态控制器、坐标系转换、时间同步、故障保护、地理围栏和安全员接管。

## 2. 快速开始

### Conda

```powershell
conda create -n los_uav python=3.11 -y
conda activate los_uav
python -m pip install -e .
```

### 单机测试

```powershell
python -m los_uav_interception single --motion line
```

### 三机测试

```powershell
python -m los_uav_interception multi --motion jink
```

默认输出：

- `outputs/trajectory.png`：三维轨迹；
- `outputs/trajectory.csv`：每帧位置和距离；
- 终端：成功标志、最近距离、仿真时间和 F 指标。

加入逐帧 `0.5°` 方位/高度角噪声和 `7.5%` 固定测距偏差：

```powershell
python -m los_uav_interception multi `
  --motion jink `
  --angle-noise-deg 0.5 `
  --range-bias 0.075
```

运行测试：

```powershell
python -m unittest discover -s tests -v
```

批量测试随机初始距离、方位和高度：

```powershell
python examples\benchmark.py single --episodes 100
python examples\benchmark.py multi --episodes 100 --angle-noise-deg 0.5
```

独立版在 `2026-07-28` 使用无观测噪声、随机初始距离/方位/高度、每类 20 回合的冒烟结果：

| 目标运动 | 单机成功率 | 三机成功率 |
|---|---:|---:|
| line | 100% | 100% |
| arc | 95% | 100% |
| multi_sine | 5% | 5% |
| jink | 65% | 75% |

这组结果只用于代码回归，不是最终算法结论。尤其是 `multi_sine`，独立版改为从连续方位观测估计 LOS-rate 后难度显著上升，说明原项目中由仿真真实相对速度合成 LOS-rate 的方式确实可能高估性能。

## 3. 控制器接口

最小调用方式见 `examples/hardware_loop.py`：

```python
command = controller.command(
    relative_position=observed_relative_position_ned_m,
    own_velocity=own_velocity_ned_mps,
    limits=AIRCRAFT_LIMITS["A"],
    dt=0.1,
)
acceleration_command_ned_mps2 = command.acceleration
```

如果外部设备已经给出 LOS 角速度，可传入：

```python
command = controller.command(
    relative_position=relative_position_ned_m,
    own_velocity=own_velocity_ned_mps,
    measured_los_rate=los_rate_rad_s,
    limits=AIRCRAFT_LIMITS["A"],
    dt=0.1,
)
```

一次任务开始前调用 `controller.reset()`，不要在每一帧重置，否则无法从连续视线估计 LOS 角速度。

## 4. LOS 的逐步流程

设观测相对位置为：

```text
r = P_target_observed - P_interceptor
l = r / ||r||
```

其中 `l` 是从拦截机指向靶机的单位视线。

### 4.1 估计视线角速度

默认实现只使用连续两帧 `l(k-1)` 和 `l(k)`。先计算两条单位视线之间的旋转轴和夹角，再除以 `dt` 得到三维 LOS 角速度 `ω_LOS`，最后使用一阶低通滤波：

```text
ω_filtered(k) = α ω_filtered(k-1) + (1-α) ω_raw(k)
```

默认 `α=0.85`。因此独立版本不通过靶机真实速度反算 LOS-rate，传感器信息边界比原仿真主线更严格。

### 4.2 预测短时视线方向

由旋转运动关系：

```text
l_dot = ω_LOS × l
aim = normalize(l + T_lead l_dot)
```

`T_lead` 默认最多 `3 s`，近距离时会自动缩短。它不是读取靶机未来轨迹，而是根据当前视线变化做短时外推。

### 4.3 追踪速度项

控制器沿 `aim` 生成满足机型速度约束的期望速度，然后用速度误差产生追踪加速度：

```text
a_pursuit = K_p (V_desired - V_own) / dt
```

默认 `K_p=0.05`。这一项提供前向追赶和低速时的稳定控制。

### 4.4 比例导航项

```text
a_PN = N |V_own| (ω_LOS × V_own_direction)
```

默认导航系数 `N=4.0`。它主要消除视线旋转，使相对运动逐渐进入近似恒定方位的碰撞航线。

### 4.5 动力学限幅

最终加速度为：

```text
a_command = limit(a_pursuit + a_PN)
```

随后点质量模型继续执行水平/垂直加速度、三轴 jerk、水平速度、爬升速度和下降速度限制。实机中应把这一步映射到飞控支持的加速度或速度设定接口。

## 5. 是否存在“作弊”

控制器本身不读取以下信息：

- 靶机类型对应的运动脚本；
- 直线、圆弧、多频 S 弯或 jink 标签；
- 靶机未来位置和航点；
- 靶机真实速度。

默认 LOS-rate 由带时间顺序的相对位置观测估计，所以不存在用仿真真值相对速度直接生成 LOS-rate 的信息泄漏。外部传入 `measured_los_rate` 时，调用方需要保证它确实来自传感器或在线估计器。

仍然可能高估实机性能的因素包括：点质量模型忽略姿态内环和执行器延迟；默认仿真无丢帧、固定延迟、误检和非高斯异常值；目标总体朝任务区推进；命中半径 `0.6 m` 对时间同步和连续碰撞检测非常敏感。

## 6. 机型参数

| 机型 | 水平速度 | 爬升/下降速度 | 水平/垂直加速度 | jerk |
|---|---:|---:|---:|---:|
| A | `20 m/s` | `8 / 6 m/s` | `3 / 2 m/s²` | `5 m/s³` |
| B | `45 m/s` | `15 / 10 m/s` | `10 / 5 m/s²` | `10 m/s³` |
| C | `20 m/s` | `8 / 6 m/s` | `3 / 2 m/s²` | `5 m/s³` |
| D | `30 m/s` | `10 / 8 m/s` | `6 / 3 m/s²` | `6 m/s³` |

代码定义在 `src/los_uav_interception/dynamics.py`。

## 7. 单机与三机

- 单机：一套传感器、一套 LOS-rate 估计器和一套控制器；
- 三机：三架机各自独立观测和闭环控制，不共享靶机未来信息；
- 三机导航系数：`3.6 / 4.0 / 4.4`；
- 三机初始速度比：`0.75 / 0.85 / 0.95`；
- 三机成功：任意一架连续轨迹最近距离小于 `0.6 m`。

三机参数不同用于形成不同到达节奏，但当前版本不是任务分配或通信协同算法。

## 8. 命中函数 F

`src/los_uav_interception/metrics.py` 记录：

- `P_v`：相对速度条件；
- `P_a`：跟踪方位角和本机速度方位角差；
- `P_e`：跟踪高度角和本机速度高度角差；
- `P_d`：距离项；
- `P=P_v P_a P_e P_d`：联合诊断值。

F 不参与 `0.6 m` 成功判定，也不反向影响 LOS 控制。

## 9. 项目结构

```text
src/los_uav_interception/
  guidance.py       LOS-rate 估计、短时视线外推、追踪项和 PN 项
  dynamics.py       A/B/C/D 参数与受速度/加速度/jerk 限制的点质量模型
  sensors.py        测距、方位角和高度角观测噪声
  simulation.py     单机/三机、目标运动、0.6 m 判定和 10 m 逃逸边界
  metrics.py        命中函数 F，仅作诊断
  plotting.py       三维轨迹和 CSV 输出
examples/
  hardware_loop.py  实机或 HIL 控制循环最小接口
  external_los_rate.py  外部 LOS-rate 输入示例
  benchmark.py      随机初始几何下的批量成功率测试
tests/              控制器、动力学和仿真回归测试
```

## 10. 与原 MAPPO 项目的关系

本仓库提取自原项目“位置 + LOS 角速度”基准控制部分，保留默认导引参数：

- `navigation_constant=4.0`；
- `pursuit_gain=0.05`；
- `lead_time=3.0 s`；
- `desired_speed_ratio=0.98`；
- `los_rate_filter_alpha=0.85`。

主要独立化调整：

1. 移除 Gymnasium、PyTorch、PPO/MAPPO 和 Actor 残差依赖；
2. 控制输入改成明确的工程接口；
3. LOS-rate 默认由连续观测视线估计，而非仿真真实相对速度；
4. 目标只存在于测试仿真，控制器不知道目标运动模式；
5. 加入可安装 Python 包、CLI、硬件接口示例和 GitHub Actions 测试。

## 11. 维护方式

日常更新：

```powershell
git pull --rebase
git status
git add src tests README.md
git commit -m "说明本次修改"
git push
```

建议每次修改控制公式时同步完成：更新 README 参数、增加或修改回归测试、运行 `python -m unittest discover -s tests -v`、至少运行一次单机和三机示例，再提交。不要把原项目的大模型权重、训练日志或生成的 `outputs/` 推到这个纯 LOS 仓库。
