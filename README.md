# LOS UAV Interception

独立的无人机 LOS（Line of Sight，视线）拦截与部署仿真项目。仓库覆盖解析 LOS 导引、A/B/C/D 飞行性能约束、机体姿态相机、观测误差、单机/三机闭环仿真、连续轨迹命中判定和控制振荡诊断。

- 当前版本：`v0.4.1`
- 当前主配置：`stable_current + accurate_delayed`
- 命中阈值：连续轨迹最近距离 `< 0.6 m`
- 正式报告：[`docs/main_aligned_deployment_20260825/REPORT.md`](docs/main_aligned_deployment_20260825/REPORT.md)
- 两仓库一致性审计：[`docs/main_aligned_deployment_20260825/PARITY.md`](docs/main_aligned_deployment_20260825/PARITY.md)
- 60 组独立三维轨迹：[`docs/main_aligned_deployment_20260825/version_trajectories/INDEX.md`](docs/main_aligned_deployment_20260825/version_trajectories/INDEX.md)

## 1. 当前系统

### 1.1 仿真与传感器

- 仿真步长 `0.1 s`，最长 `1000` 步；
- A 型拦截机、C 型靶机，速度、加速度和 jerk 均受机型约束；
- 单机初速度比例 `0.85`，三机初速度比例 `0.75/0.85/0.95`；
- 30 Hz 相机最近帧，观测延迟 `1/30 s`；
- `24° × 16°` FOV、`1%` 随机丢检、机体姿态响应时间常数 `0.25 s`；
- 方位角和高度角逐帧加入截断范围为 `±3σ` 的 `0.5°` 高斯误差；
- 测距模型使用 `D=C/N`，`C=1200 pixel·m`，像素误差为 `±1 pixel`；
- 1 帧控制延迟，实际加速度采用 `0.2 s` 一阶执行器响应；
- 目标丢失后按主项目口径衰减上一时刻 LOS-rate 并滑行；
- 六类靶机轨迹：line、cosine、arc、random、multi-sine、B 样条。

三机模式保留独立仓库的差异化角色：三架独立 LOS 拦截机的导引常数分别为基础值的 `0.9/1.0/1.1` 倍。主项目的一架观测机与两架拦截机角色结构未移植到本仓库。

### 1.2 命中与命中函数 F

相邻仿真状态之间使用连续线段最近距离，任一拦截机与靶机的最近距离小于 `0.6 m` 即判定回合成功。命中函数 F 的相对速度、方位角、高度角和距离分量只作为诊断记录，不参与成功判定。

### 1.3 Stable LOS 参数

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `navigation_gain` | `4.0` | 比例导航强度 |
| `pursuit_gain` | `0.04` | 期望速度追逐增益 |
| `lead_time` | `3.0 s` | LOS 短时提前瞄准 |
| `los_rate_soft_threshold` | `0.002 rad/s` | 抑制微小 LOS-rate 扰动 |
| `velocity_filter_tau` | `0.25 s` | 平滑期望速度 |
| `velocity_slew_limit` | `4 m/s²` | 限制期望速度变化率 |
| `acceleration_filter_tau` | `0.12 s` | 平滑导引加速度 |

## 2. LOS 控制流程

观测相对位置与视线单位向量：

```text
r = P_target_observed - P_interceptor
l = r / ||r||
```

LOS 角速度：

```text
omega_LOS = (r × v_relative) / ||r||²
```

视线方向变化率与提前瞄准方向：

```text
dl/dt = omega_LOS × l
aim = normalize(l + T_lead * dl/dt)
```

追逐项和比例导航项：

```text
a_pursuit = K_pursuit * (v_desired - v_interceptor) / dt
a_PN = N * |v_interceptor| * (omega_LOS × v_hat_interceptor)
a_base = a_pursuit + a_PN
```

`a_base` 依次经过 LOS-rate 软阈值、期望速度低通、速度变化率限制、导引加速度低通、归一化动作限幅、控制延迟、执行器一阶响应和机型 jerk 限制。

控制器不读取靶机轨迹类别、未来航点或未来控制量。`accurate_delayed` 表示外部感知/融合系统能够在保留观测误差与延迟的条件下提供准确 LOS 角速度；实机可由视觉跟踪、雷达、惯导融合或专用 LOS-rate 估计器产生该输入。

## 3. 正式复测结果

固定基准种子为 `2026082591`。每种曲线测试 `100` 回合，单机和三机各 `600` 回合。

### 3.1 当前配置

| 场景 | 曲线 | 0.6 m 命中率 | 振荡率 | 最小距离均值 | 最大轴换向频率均值 | jerk RMS | 可见帧比例 |
|---|---|---:|---:|---:|---:|---:|---:|
| 单机 | line | 100% | 0% | 0.073 m | 0.070 Hz | 0.387 m/s³ | 91.5% |
| 单机 | cosine | 89% | 0% | 0.942 m | 0.086 Hz | 0.591 m/s³ | 86.4% |
| 单机 | arc | 89% | 0% | 1.853 m | 0.073 Hz | 0.492 m/s³ | 88.3% |
| 单机 | random | 85% | 0% | 1.242 m | 0.081 Hz | 0.481 m/s³ | 84.1% |
| 单机 | multi-sine | 74% | 0% | 1.491 m | 0.080 Hz | 0.687 m/s³ | 75.8% |
| 单机 | B 样条 | 76% | 0% | 1.510 m | 0.113 Hz | 0.743 m/s³ | 76.8% |
| **单机** | **总体** | **85.50%** | **0%** | **1.185 m** | **0.084 Hz** | **0.563 m/s³** | **83.8%** |
| 三机 | line | 100% | 0% | 0.090 m | 0.076 Hz | 0.288 m/s³ | 91.7% |
| 三机 | cosine | 89% | 0% | 0.856 m | 0.096 Hz | 0.444 m/s³ | 86.3% |
| 三机 | arc | 89% | 0% | 1.770 m | 0.078 Hz | 0.356 m/s³ | 88.1% |
| 三机 | random | 86% | 0% | 1.036 m | 0.085 Hz | 0.392 m/s³ | 84.3% |
| 三机 | multi-sine | 75% | 0% | 1.059 m | 0.099 Hz | 0.595 m/s³ | 76.5% |
| 三机 | B 样条 | 80% | 0% | 1.194 m | 0.125 Hz | 0.661 m/s³ | 78.7% |
| **三机** | **总体** | **86.50%** | **0%** | **1.001 m** | **0.093 Hz** | **0.456 m/s³** | **84.3%** |

三机相对单机提高 `1.00` 个百分点。当前三机使用相邻初始位置和相近控制参数，各机失败事件相关性较高；额外收益主要来自 random、multi-sine 和 B 样条，尚未形成三个统计独立的拦截机会。

### 3.2 五个 LOS 版本

历史四个版本使用 `bearing_history`，当前版本使用 `accurate_delayed`。所有版本采用同一 v0.4.1 环境、种子、FOV、噪声和动力学。

| 场景 | 版本 | 命中率 | 振荡率 | 换向频率均值 | jerk RMS |
|---|---|---:|---:|---:|---:|
| 单机 | legacy | 72.17% | 87.17% | 3.230 Hz | 3.671 m/s³ |
| 单机 | stable_v02 | 39.17% | 4.33% | 0.123 Hz | 0.932 m/s³ |
| 单机 | conservative | 26.83% | 0% | 0.043 Hz | 0.652 m/s³ |
| 单机 | flight_test | 19.00% | 0% | 0.027 Hz | 0.556 m/s³ |
| 单机 | **stable_current** | **85.50%** | **0%** | **0.084 Hz** | **0.563 m/s³** |
| 三机 | legacy | 78.67% | 88.50% | 3.713 Hz | 3.775 m/s³ |
| 三机 | stable_v02 | 64.00% | 14.33% | 0.213 Hz | 1.028 m/s³ |
| 三机 | conservative | 46.00% | 0.17% | 0.071 Hz | 0.693 m/s³ |
| 三机 | flight_test | 24.17% | 0% | 0.032 Hz | 0.566 m/s³ |
| 三机 | **stable_current** | **86.50%** | **0%** | **0.093 Hz** | **0.456 m/s³** |

当前配置相对 legacy：单机命中率提高 `13.33` 个百分点、jerk RMS 降低 `84.7%`；三机命中率提高 `7.83` 个百分点、jerk RMS 降低 `87.9%`。

六曲线逐项结果采用“命中率/振荡率”表示：

| 单机版本 | line | cosine | arc | random | multi-sine | B 样条 |
|---|---:|---:|---:|---:|---:|---:|
| legacy | 97/92 | 75/87 | 84/89 | 78/85 | 53/84 | 46/86 |
| stable_v02 | 82/3 | 42/4 | 39/10 | 46/5 | 15/1 | 11/3 |
| conservative | 67/0 | 17/0 | 35/0 | 26/0 | 7/0 | 9/0 |
| flight_test | 91/0 | 0/0 | 7/0 | 8/0 | 7/0 | 1/0 |
| **stable_current** | **100/0** | **89/0** | **89/0** | **85/0** | **74/0** | **76/0** |

| 三机版本 | line | cosine | arc | random | multi-sine | B 样条 |
|---|---:|---:|---:|---:|---:|---:|
| legacy | 100/93 | 85/88 | 89/89 | 82/86 | 61/85 | 55/90 |
| stable_v02 | 99/7 | 76/17 | 75/35 | 72/22 | 39/2 | 23/3 |
| conservative | 92/0 | 39/0 | 70/1 | 39/0 | 19/0 | 17/0 |
| flight_test | 100/0 | 1/0 | 15/0 | 12/0 | 11/0 | 6/0 |
| **stable_current** | **100/0** | **89/0** | **89/0** | **86/0** | **75/0** | **80/0** |

### 3.3 LOS-rate 输入消融

| 场景 | 参数 | LOS-rate 来源 | 命中率 | 振荡率 | jerk RMS |
|---|---|---|---:|---:|---:|
| 单机 | v0.2 stable | 方位历史 | 39.17% | 4.33% | 0.932 m/s³ |
| 单机 | v0.2 stable | 精确延迟 LOS-rate | 85.33% | 0.33% | 0.716 m/s³ |
| 单机 | 当前 stable | 方位历史 | 39.33% | 88.67% | 3.398 m/s³ |
| 单机 | 当前 stable | 精确延迟 LOS-rate | 85.50% | 0% | 0.563 m/s³ |
| 三机 | v0.2 stable | 方位历史 | 64.00% | 14.33% | 1.028 m/s³ |
| 三机 | v0.2 stable | 精确延迟 LOS-rate | 86.83% | 0.33% | 0.656 m/s³ |
| 三机 | 当前 stable | 方位历史 | 64.00% | 91.50% | 3.784 m/s³ |
| 三机 | 当前 stable | 精确延迟 LOS-rate | 86.50% | 0% | 0.456 m/s³ |

主要命中提升来自 LOS-rate 输入质量。当前低延迟滤波参数针对低噪声外部 LOS-rate 标定，直接接入带 `0.5°` 噪声的方位历史差分会放大高频换向。方位历史接口应采用 `stable_v02`，或在部署前重新设计 LOS-rate 估计器和滤波参数。

## 4. 振荡指标

每回合去除前 `2 s` 暖机数据。单架飞机同时满足以下条件时判定为振荡：

1. 有效时长不少于 `4 s`；
2. 归一化三轴动作越过 `0.08` 死区后的最大轴换向频率不低于 `0.4 Hz`；
3. 实际加速度在 `0.2–2.5 Hz` 频带的动态能量占比不低于 `0.25`；
4. 实际加速度最大轴峰峰值不低于 `0.8 m/s²`。

多机中任一拦截机同时满足全部条件，该回合即记为振荡。当前单机换向频率中位数/95 分位/最大值为 `0.080/0.217/0.366 Hz`，三机为 `0.086/0.235/0.380 Hz`，全部低于 `0.4 Hz`，因此正式样本中的联合振荡率为 `0%`。600 回合零检出对应的二项分布近似 95% 上界约为 `0.5%`，实机仍需用相同字段和判据复核。

汇总文件中的 `*_threshold_exceedance_rate` 表示单项指标超过阈值的比例，不用于表示安全通过率。最终振荡率只统计三个动态阈值与有效时长同时满足的回合。

## 5. 安装与运行

```powershell
conda create -n los_uav python=3.11 -y
conda activate los_uav
python -m pip install -e .
```

```powershell
# 当前单机配置
python -m los_uav_interception single --motion cosine

# 当前三机配置
python -m los_uav_interception multi --motion bspline

# 方位历史 + v0.2 稳定参数
python -m los_uav_interception single `
  --motion random `
  --los-rate-source bearing_history `
  --guidance-profile stable_v02
```

默认生成 `outputs/trajectory.png` 和 `outputs/trajectory.csv`。CSV 包含位置、目标可见状态、测距误差、期望速度、归一化动作和实际执行加速度。

## 6. Python 接口

```python
from los_uav_interception import AIRCRAFT_LIMITS, LOSGuidanceConfig, LOSGuidanceController

controller = LOSGuidanceController(LOSGuidanceConfig())
command = controller.command(
    relative_position=observed_relative_position_ned_m,
    own_velocity=own_velocity_ned_mps,
    measured_los_rate=measured_los_rate_rad_s,
    limits=AIRCRAFT_LIMITS["A"],
    dt=0.1,
)

acceleration_command_ned_mps2 = command.acceleration
normalized_action = command.normalized_action
```

任务开始前调用一次 `controller.reset()`。目标丢失时通过 `GuidanceSafetyGate` 和 `controller.coast(dt)` 延续主项目的衰减滑行逻辑。

## 7. 数据与复现

```powershell
python -m unittest discover -s tests -v

python examples\stability_benchmark.py `
  --scope single --episodes 100 --seed 2026082591 `
  --los-rate-source accurate_delayed --profiles stable `
  --output docs\main_aligned_deployment_20260825\single_episodes.csv
```

主要数据：

- [`single_episodes.csv`](docs/main_aligned_deployment_20260825/single_episodes.csv)
- [`multi_episodes.csv`](docs/main_aligned_deployment_20260825/multi_episodes.csv)
- [`comparison_versions_summary.csv`](docs/main_aligned_deployment_20260825/comparison_versions_summary.csv)
- [`ablation_los_source_summary.csv`](docs/main_aligned_deployment_20260825/ablation_los_source_summary.csv)
- [`version_trajectories/trajectory_index.csv`](docs/main_aligned_deployment_20260825/version_trajectories/trajectory_index.csv)

## 8. 工程边界

当前验证范围为导引层、观测链路和简化执行器。实机部署仍需完成姿态角速度内环、真实电机/桨动力学、NED/机体/相机/云台坐标标定、时间戳同步、通信抖动、风扰、GPS/惯导误差、目标跟踪置信度、地理围栏、故障保护、HIL 和分级限速试飞。

## 9. 项目结构

```text
src/los_uav_interception/
  dynamics.py       机型速度、加速度和 jerk 约束
  sensors.py        像素测距、角噪声、机体姿态相机和 FOV
  guidance.py       LOS-rate、追逐项、PN 项、滤波和限幅
  integration.py    时间戳、失帧和输入有效性安全门
  simulation.py     六类靶机轨迹、单机/三机闭环仿真
  stability.py      与主项目一致的振荡指标
  metrics.py        命中函数 F 诊断
  plotting.py       三维轨迹和逐帧 CSV
examples/
  stability_benchmark.py  批量命中/振荡测试
  hardware_loop.py        HIL/实机调用示例
docs/main_aligned_deployment_20260825/
  REPORT.md                正式报告
  PARITY.md                两仓库一致性审计
  version_trajectories/    60 组版本轨迹图和 CSV
tests/                     回归测试
```

## 10. Git 维护

```powershell
git status
git pull --ff-only origin main
git add README.md CHANGELOG.md src tests docs
git commit -m "describe change"
git push origin main
```

分支名称使用功能或版本语义，例如 `stable-los-validation`、`release-0.4.2`。控制参数、观测模型、动力学、命中阈值或振荡判据发生变化时，应同步更新版本号、`CHANGELOG.md`、固定种子六曲线结果、原始 CSV 和代表轨迹图。
