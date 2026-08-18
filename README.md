# LOS UAV Interception

这是从 `mappo_uav_project` 中独立出来的纯 LOS（Line of Sight，视线）无人机拦截项目。它不包含 PPO/MAPPO、Actor、Critic 或模型权重，只保留可直接部署和测试的解析导引、受限点质量动力学、传感器噪声、单机/三机仿真与命中函数 F 诊断。

> **2026-08-18 / v0.2.0：**针对实机速度指令震荡，新增滑窗 LOS-rate 回归、LOS 角速度软阈值、期望速度低通与变化率限制、加速度低通、失帧/超时安全门、`24°×16°` FOV 门控、像素量化测距和震荡诊断。正式测试统一使用六类目标轨迹。完整原理、参数、分轨迹测试数据和实机限制见 [`docs/STABILIZATION_2026-08-18.md`](docs/STABILIZATION_2026-08-18.md)。

## 1. 适用范围

- 输入：本机 NED 速度、靶机相对本机的 NED 位置；
- 可选输入：雷达、视觉跟踪器或状态估计器给出的 LOS 角速度；
- 输出：本机在 NED 坐标系下的三轴加速度指令，单位 `m/s²`；
- 不需要：靶机运动模式、未来航点、靶机控制指令或靶机真实速度；
- 仿真成功判定：连续轨迹最近距离小于 `0.6 m`；
- 命中函数 F：只记录诊断指标，不决定是否成功。

默认观测模型包含逐帧 `0.5°` 方位角/高度角高斯误差、`C=1200 pixel·m` 的像素量化测距、`±1 pixel` 检测误差和 `24°×16°` FOV。FOV 决定目标是否可见，像素量化决定目标可见时的距离误差，两者独立执行。由于点质量模型没有机体姿态和云台状态，FOV 光轴暂以拦截机速度方向近似；目标出视场后导引释放，拦截机保持当前速度滑行，重新进入视场后需一帧恢复 LOS-rate 估计。

像素测距使用：

```text
N_true = C / D_true
N_measured = max(1, round(N_true) + e_pixel),  e_pixel ∈ {-1, 0, +1}
D_observed = C / N_measured
```

`D_true=300 m` 时目标约为 `4 pixel`。测得 `3/4/5 pixel` 时，距离分别为 `400/300/240 m`，对应 `+33.3%/0%/-20%`；距离继续增加时像素数更少，相对误差会进一步增大。

本仓库定位为导航与算法验证软件。完整飞行系统还需配置姿态控制器、坐标系转换、时间同步、故障保护、地理围栏和安全员接管功能。

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
python -m los_uav_interception multi --motion bspline
```

默认输出：

- `outputs/trajectory.png`：三维轨迹；
- `outputs/trajectory.csv`：每帧位置和距离；
- 终端：成功标志、最近距离、仿真时间和 F 指标。

默认命令已经启用观测误差与 FOV。以下命令显式给出相同参数：

```powershell
python -m los_uav_interception multi `
  --motion bspline `
  --guidance-profile stable `
  --angle-noise-deg 0.5 `
  --camera-constant-pixel-m 1200 `
  --pixel-error-max 1 `
  --fov-horizontal-deg 24 `
  --fov-vertical-deg 16
```

`--no-fov` 仅用于 FOV 消融测试。

运行测试：

```powershell
python -m unittest discover -s tests -v
```

批量测试随机初始距离、方位和高度：

```powershell
python examples\benchmark.py single --episodes 100
python examples\benchmark.py multi --episodes 100 --angle-noise-deg 0.5
```

对比原始、均衡稳定化、保守和实机首飞配置的成功率与震荡率：

```powershell
python examples\stability_benchmark.py --scope single --episodes 100
python examples\stability_benchmark.py --scope multi --episodes 50
```

`2026-08-18` 正式测试采用 `line / cosine / arc / random / multi_sine / bspline` 六类目标导向轨迹、逐帧 `0.5°` 角噪声、像素量化测距和 `24°×16°` FOV。单机每种配置测试 600 回合，三机每种配置测试 300 回合。

配置含义：

- `legacy`（原始配置）：v0.1.0 两帧差分和未整形导引指令，用于复现旧 LOS；
- `stable`（均衡稳定化）：滑窗 LOS-rate、软阈值和中等强度指令整形，兼顾响应与平滑；
- `conservative`（增强滤波）：提高 LOS-rate 与加速度滤波强度，降低命令换向和 jerk；
- `flight_test`（首飞低变化率）：使用最强滤波与最低速度指令变化率，用于低速 HIL 和受控首飞。

指标定义：

- **0.6 m 命中率**：回合内连续轨迹最近距离小于 `0.6 m` 的回合比例；
- **振荡回合占比**：同时满足持续时间、指令换向频率、频带能量和速度误差峰峰值阈值的回合比例；
- **最大轴指令换向频率均值**：每回合分别统计 N/E/D 三轴加速度指令越过死区后的符号换向频率，取三轴最大值，再对全部回合求均值；
- **加速度指令 jerk RMS**：滤波和限幅后的加速度指令差分除以 `dt`，计算三轴向量模的均方根。该值描述指令变化强度，未包含真实飞控和机体响应；
- **单机 FOV 内帧比例**：对全部拦截机和全部仿真帧统计目标位于相机视场内的比例；
- **全系统丢失帧比例**：该帧没有任何拦截机看见目标的比例；
- **平均绝对测距误差**：仅对目标位于 FOV 内且产生有效距离观测的帧，统计 `|D_observed/D_true-1|` 的均值；
- **测距误差超过 20% 帧比例**：有效距离观测中绝对相对误差大于 `20%` 的比例；
- 三机场景的命中条件为任一拦截机命中，振荡条件为任一拦截机满足振荡判据。三机换向频率取三架机最大值，jerk RMS 取三架机均值；
- 命中与振荡为独立指标，同一回合可以同时命中并被判定为振荡。

单机分轨迹结果：

| 配置 | 轨迹 | 命中率 | 振荡率 | 全系统丢失帧 | 测距 MAE | 误差>20%帧 |
|---|---|---:|---:|---:|---:|---:|
| legacy | line | 99% | 98% | 0.7% | 13.4% | 22.7% |
| legacy | cosine | 0% | 100% | 58.7% | 14.9% | 25.8% |
| legacy | arc | 95% | 99% | 2.8% | 13.5% | 23.2% |
| legacy | random | 94% | 99% | 3.4% | 13.4% | 22.9% |
| legacy | multi_sine | 22% | 98% | 41.5% | 13.7% | 23.8% |
| legacy | bspline | 78% | 98% | 11.5% | 13.7% | 23.6% |
| stable | line | 92% | 11% | 6.1% | 14.0% | 24.2% |
| stable | cosine | 0% | 1% | 61.2% | 15.6% | 27.6% |
| stable | arc | 65% | 58% | 20.9% | 14.0% | 25.1% |
| stable | random | 78% | 34% | 13.3% | 13.8% | 24.3% |
| stable | multi_sine | 12% | 5% | 47.9% | 14.1% | 24.5% |
| stable | bspline | 53% | 20% | 26.7% | 14.3% | 24.3% |
| conservative | line | 71% | 0% | 16.2% | 13.7% | 23.6% |
| conservative | cosine | 0% | 0% | 58.9% | 15.0% | 26.0% |
| conservative | arc | 57% | 5% | 23.3% | 13.8% | 24.2% |
| conservative | random | 23% | 0% | 42.9% | 14.0% | 24.2% |
| conservative | multi_sine | 3% | 0% | 53.2% | 14.2% | 24.9% |
| conservative | bspline | 32% | 0% | 36.3% | 14.0% | 24.0% |
| flight_test | line | 90% | 0% | 5.8% | 13.6% | 23.0% |
| flight_test | cosine | 0% | 0% | 58.5% | 15.0% | 25.8% |
| flight_test | arc | 15% | 0% | 44.5% | 13.6% | 23.3% |
| flight_test | random | 6% | 0% | 49.8% | 13.6% | 23.3% |
| flight_test | multi_sine | 1% | 0% | 54.0% | 14.3% | 25.2% |
| flight_test | bspline | 4% | 0% | 50.5% | 14.0% | 23.9% |

三机分轨迹结果：

| 配置 | 轨迹 | 命中率 | 振荡率 | 全系统丢失帧 | 测距 MAE | 误差>20%帧 |
|---|---|---:|---:|---:|---:|---:|
| legacy | line | 100% | 100% | 0.0% | 13.7% | 22.9% |
| legacy | cosine | 0% | 100% | 56.8% | 14.5% | 24.7% |
| legacy | arc | 100% | 100% | 0.0% | 13.2% | 22.3% |
| legacy | random | 100% | 100% | 0.0% | 13.4% | 22.3% |
| legacy | multi_sine | 24% | 100% | 40.0% | 13.4% | 22.4% |
| legacy | bspline | 86% | 100% | 7.1% | 13.3% | 22.7% |
| stable | line | 100% | 38% | 0.0% | 13.7% | 23.0% |
| stable | cosine | 0% | 2% | 59.9% | 15.3% | 26.6% |
| stable | arc | 98% | 96% | 1.2% | 13.2% | 22.5% |
| stable | random | 100% | 78% | 0.0% | 13.4% | 22.4% |
| stable | multi_sine | 26% | 14% | 39.5% | 13.5% | 22.7% |
| stable | bspline | 84% | 66% | 8.3% | 13.5% | 22.7% |
| conservative | line | 86% | 0% | 7.2% | 13.7% | 23.0% |
| conservative | cosine | 0% | 0% | 58.2% | 14.7% | 24.8% |
| conservative | arc | 92% | 24% | 4.5% | 13.2% | 22.5% |
| conservative | random | 64% | 6% | 18.6% | 13.4% | 22.4% |
| conservative | multi_sine | 8% | 0% | 49.3% | 13.5% | 22.8% |
| conservative | bspline | 50% | 2% | 26.6% | 13.4% | 22.7% |
| flight_test | line | 98% | 2% | 1.0% | 13.7% | 22.9% |
| flight_test | cosine | 0% | 0% | 58.6% | 14.7% | 24.9% |
| flight_test | arc | 18% | 0% | 42.9% | 13.2% | 22.3% |
| flight_test | random | 12% | 0% | 46.6% | 13.5% | 22.5% |
| flight_test | multi_sine | 10% | 0% | 48.2% | 13.6% | 22.9% |
| flight_test | bspline | 12% | 2% | 46.2% | 13.5% | 22.9% |

整体指令平滑性结果：

| 场景 | 配置 | 0.6 m 命中率 | 振荡回合占比 | 最大轴换向频率 | jerk RMS |
|---|---|---:|---:|---:|---:|
| 单机 | legacy | 64.67% | 98.67% | 3.656 Hz | 23.279 m/s³ |
| 单机 | stable | 50.00% | 21.50% | 0.276 Hz | 2.378 m/s³ |
| 单机 | conservative | 31.00% | 0.83% | 0.105 Hz | 1.950 m/s³ |
| 单机 | flight_test | 19.33% | 0.00% | 0.073 Hz | 1.824 m/s³ |
| 三机 | legacy | 68.33% | 100.00% | 4.072 Hz | 23.684 m/s³ |
| 三机 | stable | 68.00% | 49.00% | 0.427 Hz | 2.380 m/s³ |
| 三机 | conservative | 50.00% | 5.33% | 0.170 Hz | 1.989 m/s³ |
| 三机 | flight_test | 25.00% | 0.67% | 0.099 Hz | 1.784 m/s³ |

结果表明，像素量化模型下约 `22%～28%` 的有效观测帧存在超过 `20%` 的测距误差。距离比例误差对归一化 LOS 方向影响有限，但会影响提前时间、目标重捕获和速度追踪项。`stable` 相对 `legacy` 显著降低命令换向频率和 jerk RMS；cosine 与 multi_sine 的低命中率主要伴随长时间 FOV 丢失。默认 `stable` 用于算法仿真，`LOSGuidanceConfig.flight_test()` 用于低速、开阔场地、具备人工接管条件的首轮 HIL 和实飞验证。

## 3. 控制器接口

实机/HIL 应使用 `examples/hardware_loop.py` 中的 `GuidanceSafetyGate`：它会拒绝首帧、丢失目标、时间戳异常、超过 `0.2 s` 的陈旧观测和不合理采样周期，并返回 `None` 让飞控进入自身安全模式。

以下直接调用只适合仿真或上层已经完成观测有效性检查的系统：

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

原始 v0.1.0 只使用相邻两帧 LOS 差分。`0.5°` 独立逐帧角噪声在 `10 Hz` 下会被微分放大，因此 v0.2.0 默认保存最近 11 帧，用最小二乘直线拟合当前 LOS 方向和变化率，再计算：

```text
l_dot = least_squares_slope(l_history, t_history)
ω_raw = l_fitted × l_dot
```

拟合值外推到当前采样时刻，不使用居中窗口的历史中点，因此比简单滑动平均具有更小的相位滞后。之后仍使用一阶低通：

```text
ω_filtered(k) = α ω_filtered(k-1) + (1-α) ω_raw(k)
```

默认滑窗为 `11` 帧、`α=0.75`。原始两帧实现可通过 `LOSGuidanceConfig.legacy()` 复现。独立版本不通过靶机真实速度反算 LOS-rate，传感器信息边界比原仿真主线更严格。

### 4.2 预测短时视线方向

由旋转运动关系：

```text
l_dot = ω_LOS × l
aim = normalize(l + T_lead l_dot)
```

`T_lead` 默认上限为 `3 s`，近距离时自动缩短。短时外推仅使用当前及历史视线观测，不使用靶机未来轨迹。

### 4.3 追踪速度项

控制器沿 `aim` 生成满足机型速度约束的期望速度，然后用速度误差产生追踪加速度：

```text
a_pursuit = K_p (V_desired - V_own) / dt
```

均衡配置默认 `K_p=0.06`。这一项提供前向追赶和低速时的稳定控制。

### 4.4 比例导航项

```text
a_PN = N |V_own| (ω_LOS × V_own_direction)
```

均衡配置默认导航系数 `N=8.0`，用于补偿滑窗和低通带来的相位滞后。它主要消除视线旋转，使相对运动逐渐进入近似恒定方位的碰撞航线。

### 4.5 抗震荡链路

在保留传感器误差的情况下依次执行：

1. `0.01 rad/s` LOS-rate 软阈值，去除围绕零点反复变号的小幅噪声；
2. 期望速度一阶低通，默认时间常数 `0.30 s`；
3. 期望速度变化率限制，默认 `4.0 m/s²`；
4. 合成加速度一阶低通，默认时间常数 `0.20 s`；
5. 飞机速度、加速度和 jerk 物理限幅。

控制结构由 LOS 一阶提前瞄准、速度误差追踪项、PN 项和带状态指令整形组成。导引层未设置误差积分状态；低通滤波器和变化率限制器负责约束观测噪声进入速度内环的幅值与频率。

### 4.6 动力学限幅

最终加速度为：

```text
a_command = limit(a_pursuit + a_PN)
```

随后点质量模型继续执行水平/垂直加速度、三轴 jerk、水平速度、爬升速度和下降速度限制。实机中应把这一步映射到飞控支持的加速度或速度设定接口。

## 5. 信息使用边界

控制器输入不包含以下信息：

- 靶机类型对应的运动脚本；
- 直线、圆弧、多频 S 弯或 jink 标签；
- 靶机未来位置和航点；
- 靶机真实速度。

默认 LOS-rate 由带时间顺序的相对位置观测估计，控制过程不调用仿真真值相对速度。外部 `measured_los_rate` 输入应来自传感器或在线状态估计器，并保留对应时间戳和有效性标志。

仍然可能高估实机性能的因素包括：点质量模型忽略姿态内环和执行器延迟；默认仿真无丢帧、固定延迟、误检和非高斯异常值；目标总体朝任务区推进；命中半径 `0.6 m` 对时间同步和连续碰撞检测非常敏感。

## 6. 实机部署边界与安全流程

本仓库的软件成熟度为飞控外环导引研究原型，尚未完成适航或飞行安全认证。当前仿真未完整包含姿态环、速度 PI、积分饱和、机架弹性、电机/舵机延迟、通信抖动、相机曝光与检测延迟、滚转造成的 FOV 丢失、GPS/惯导坐标偏差和多机链路异常。

推荐流程：

1. 软件在环：使用实测时间戳、噪声、丢帧和延迟日志回放；
2. 硬件在环：确认 NED/ENU/机体系符号、单位、时间同步和失效模式；
3. 系留或低速首飞：使用 `LOSGuidanceConfig.flight_test()`，速度环积分先从较小值开始，并启用 anti-windup；
4. 记录 `LOS`、`LOS-rate`、原始/滤波期望速度、PN/追踪加速度、实际速度、姿态角和电机输出；
5. 只有在速度内环无持续反转、目标不出 FOV、人工接管可靠后，才逐步切换到均衡配置。

`GuidanceSafetyGate` 返回 `None` 时，上层飞控必须切回悬停、定速、返航或人工接管等经过验证的安全模式，不能把 `None` 当作零油门或零姿态指令。

CLI 可通过 `--guidance-profile legacy|stable|conservative|flight_test` 切换档位；Python 接口分别使用 `LOSGuidanceConfig.legacy()`、`LOSGuidanceConfig()`、`LOSGuidanceConfig.conservative()` 和 `LOSGuidanceConfig.flight_test()`。

## 7. 机型参数

| 机型 | 水平速度 | 爬升/下降速度 | 水平/垂直加速度 | jerk |
|---|---:|---:|---:|---:|
| A | `20 m/s` | `8 / 6 m/s` | `3 / 2 m/s²` | `5 m/s³` |
| B | `45 m/s` | `15 / 10 m/s` | `10 / 5 m/s²` | `10 m/s³` |
| C | `20 m/s` | `8 / 6 m/s` | `3 / 2 m/s²` | `5 m/s³` |
| D | `30 m/s` | `10 / 8 m/s` | `6 / 3 m/s²` | `6 m/s³` |

代码定义在 `src/los_uav_interception/dynamics.py`。

## 8. 单机与三机

- 单机：一套传感器、一套 LOS-rate 估计器和一套控制器；
- 三机：三架机各自独立观测和闭环控制，不共享靶机未来信息；
- 三机导航系数：当前配置导航系数的 `0.9 / 1.0 / 1.1` 倍；
- 三机初始速度比：`0.75 / 0.85 / 0.95`；
- 三机成功：任意一架连续轨迹最近距离小于 `0.6 m`。

三机参数差异用于形成不同到达节奏。当前版本采用三套独立导引控制器，未包含任务分配和通信协同模块。

## 9. 命中函数 F

`src/los_uav_interception/metrics.py` 记录：

- `P_v`：相对速度条件；
- `P_a`：跟踪方位角和本机速度方位角差；
- `P_e`：跟踪高度角和本机速度高度角差；
- `P_d`：距离项；
- `P=P_v P_a P_e P_d`：联合诊断值。

F 不参与 `0.6 m` 成功判定，也不反向影响 LOS 控制。

## 10. 项目结构

```text
src/los_uav_interception/
  guidance.py       LOS-rate 估计、短时视线外推、追踪项和 PN 项
  integration.py    实机观测时效、丢失目标和采样周期安全门
  stability.py      指令反转、频带能量和 jerk 震荡诊断
  dynamics.py       A/B/C/D 参数与受速度/加速度/jerk 限制的点质量模型
  sensors.py        测距、方位角和高度角观测噪声
  simulation.py     单机/三机、目标运动、0.6 m 判定和 10 m 逃逸边界
  metrics.py        命中函数 F，仅作诊断
  plotting.py       三维轨迹和 CSV 输出
examples/
  hardware_loop.py  实机或 HIL 控制循环最小接口
  external_los_rate.py  外部 LOS-rate 输入示例
  benchmark.py      随机初始几何下的批量成功率测试
  stability_benchmark.py  原始/稳定化配置成功率与震荡率对比
tests/              控制器、动力学和仿真回归测试
```

## 11. 与原 MAPPO 项目的关系

原始 v0.1.0 保留了 MAPPO 项目旧 LOS 参数，可通过 `LOSGuidanceConfig.legacy()` 复现：

- `navigation_constant=4.0`；
- `pursuit_gain=0.05`；
- `lead_time=3.0 s`；
- `desired_speed_ratio=0.98`；
- `los_rate_filter_alpha=0.85`。

v0.2.0 默认均衡稳定化参数为：

- `navigation_constant=8.0`；
- `pursuit_gain=0.06`；
- `lead_time=3.0 s`；
- `los_rate_window_size=11`；
- `los_rate_filter_alpha=0.75`；
- `los_rate_soft_threshold_rad_s=0.01`；
- `desired_velocity_filter_tau_s=0.30`；
- `desired_velocity_slew_limit_mps2=4.0`；
- `acceleration_filter_tau_s=0.20`。

主要独立化调整：

1. 移除 Gymnasium、PyTorch、PPO/MAPPO 和 Actor 残差依赖；
2. 控制输入改成明确的工程接口；
3. LOS-rate 默认由连续观测视线估计，而非仿真真实相对速度；
4. 目标只存在于测试仿真，控制器不知道目标运动模式；
5. 加入可安装 Python 包、CLI、硬件接口示例和 GitHub Actions 测试。

## 12. 维护方式

日常更新：

```powershell
git pull --rebase
git status
git add src tests README.md
git commit -m "说明本次修改"
git push
```

建议每次修改控制公式时同步完成：更新 README 参数、增加或修改回归测试、运行 `python -m unittest discover -s tests -v`、至少运行一次单机和三机示例，再提交。不要把原项目的大模型权重、训练日志或生成的 `outputs/` 推到这个纯 LOS 仓库。
