# v0.4.1 三维轨迹图索引

本目录包含 `2` 种场景、`5` 个 LOS 版本和 `6` 类靶机轨迹，共 `60` 张独立三维轨迹图与 `60` 份逐帧 CSV。每个示例的随机种子、命中结果和最小距离记录在 [`trajectory_index.csv`](trajectory_index.csv)。

历史四个版本使用 `bearing_history`，当前稳定版本使用 `accurate_delayed`。图像标题包含命中结果、最小距离、目标可见比例和测距平均绝对误差。

## 单机

### legacy + bearing_history

| line | cosine | arc | random | multi-sine | B 样条 |
|---|---|---|---|---|---|
| [图](single/legacy_history/line.png) / [CSV](single/legacy_history/line.csv) | [图](single/legacy_history/cosine.png) / [CSV](single/legacy_history/cosine.csv) | [图](single/legacy_history/arc.png) / [CSV](single/legacy_history/arc.csv) | [图](single/legacy_history/random.png) / [CSV](single/legacy_history/random.csv) | [图](single/legacy_history/multi_sine.png) / [CSV](single/legacy_history/multi_sine.csv) | [图](single/legacy_history/bspline.png) / [CSV](single/legacy_history/bspline.csv) |

### stable_v02 + bearing_history

| line | cosine | arc | random | multi-sine | B 样条 |
|---|---|---|---|---|---|
| [图](single/stable_v02_history/line.png) / [CSV](single/stable_v02_history/line.csv) | [图](single/stable_v02_history/cosine.png) / [CSV](single/stable_v02_history/cosine.csv) | [图](single/stable_v02_history/arc.png) / [CSV](single/stable_v02_history/arc.csv) | [图](single/stable_v02_history/random.png) / [CSV](single/stable_v02_history/random.csv) | [图](single/stable_v02_history/multi_sine.png) / [CSV](single/stable_v02_history/multi_sine.csv) | [图](single/stable_v02_history/bspline.png) / [CSV](single/stable_v02_history/bspline.csv) |

### conservative + bearing_history

| line | cosine | arc | random | multi-sine | B 样条 |
|---|---|---|---|---|---|
| [图](single/conservative_history/line.png) / [CSV](single/conservative_history/line.csv) | [图](single/conservative_history/cosine.png) / [CSV](single/conservative_history/cosine.csv) | [图](single/conservative_history/arc.png) / [CSV](single/conservative_history/arc.csv) | [图](single/conservative_history/random.png) / [CSV](single/conservative_history/random.csv) | [图](single/conservative_history/multi_sine.png) / [CSV](single/conservative_history/multi_sine.csv) | [图](single/conservative_history/bspline.png) / [CSV](single/conservative_history/bspline.csv) |

### flight_test + bearing_history

| line | cosine | arc | random | multi-sine | B 样条 |
|---|---|---|---|---|---|
| [图](single/flight_test_history/line.png) / [CSV](single/flight_test_history/line.csv) | [图](single/flight_test_history/cosine.png) / [CSV](single/flight_test_history/cosine.csv) | [图](single/flight_test_history/arc.png) / [CSV](single/flight_test_history/arc.csv) | [图](single/flight_test_history/random.png) / [CSV](single/flight_test_history/random.csv) | [图](single/flight_test_history/multi_sine.png) / [CSV](single/flight_test_history/multi_sine.csv) | [图](single/flight_test_history/bspline.png) / [CSV](single/flight_test_history/bspline.csv) |

### stable_current + accurate_delayed

| line | cosine | arc | random | multi-sine | B 样条 |
|---|---|---|---|---|---|
| [图](single/stable_current_accurate/line.png) / [CSV](single/stable_current_accurate/line.csv) | [图](single/stable_current_accurate/cosine.png) / [CSV](single/stable_current_accurate/cosine.csv) | [图](single/stable_current_accurate/arc.png) / [CSV](single/stable_current_accurate/arc.csv) | [图](single/stable_current_accurate/random.png) / [CSV](single/stable_current_accurate/random.csv) | [图](single/stable_current_accurate/multi_sine.png) / [CSV](single/stable_current_accurate/multi_sine.csv) | [图](single/stable_current_accurate/bspline.png) / [CSV](single/stable_current_accurate/bspline.csv) |

## 三机

### legacy + bearing_history

| line | cosine | arc | random | multi-sine | B 样条 |
|---|---|---|---|---|---|
| [图](multi/legacy_history/line.png) / [CSV](multi/legacy_history/line.csv) | [图](multi/legacy_history/cosine.png) / [CSV](multi/legacy_history/cosine.csv) | [图](multi/legacy_history/arc.png) / [CSV](multi/legacy_history/arc.csv) | [图](multi/legacy_history/random.png) / [CSV](multi/legacy_history/random.csv) | [图](multi/legacy_history/multi_sine.png) / [CSV](multi/legacy_history/multi_sine.csv) | [图](multi/legacy_history/bspline.png) / [CSV](multi/legacy_history/bspline.csv) |

### stable_v02 + bearing_history

| line | cosine | arc | random | multi-sine | B 样条 |
|---|---|---|---|---|---|
| [图](multi/stable_v02_history/line.png) / [CSV](multi/stable_v02_history/line.csv) | [图](multi/stable_v02_history/cosine.png) / [CSV](multi/stable_v02_history/cosine.csv) | [图](multi/stable_v02_history/arc.png) / [CSV](multi/stable_v02_history/arc.csv) | [图](multi/stable_v02_history/random.png) / [CSV](multi/stable_v02_history/random.csv) | [图](multi/stable_v02_history/multi_sine.png) / [CSV](multi/stable_v02_history/multi_sine.csv) | [图](multi/stable_v02_history/bspline.png) / [CSV](multi/stable_v02_history/bspline.csv) |

### conservative + bearing_history

| line | cosine | arc | random | multi-sine | B 样条 |
|---|---|---|---|---|---|
| [图](multi/conservative_history/line.png) / [CSV](multi/conservative_history/line.csv) | [图](multi/conservative_history/cosine.png) / [CSV](multi/conservative_history/cosine.csv) | [图](multi/conservative_history/arc.png) / [CSV](multi/conservative_history/arc.csv) | [图](multi/conservative_history/random.png) / [CSV](multi/conservative_history/random.csv) | [图](multi/conservative_history/multi_sine.png) / [CSV](multi/conservative_history/multi_sine.csv) | [图](multi/conservative_history/bspline.png) / [CSV](multi/conservative_history/bspline.csv) |

### flight_test + bearing_history

| line | cosine | arc | random | multi-sine | B 样条 |
|---|---|---|---|---|---|
| [图](multi/flight_test_history/line.png) / [CSV](multi/flight_test_history/line.csv) | [图](multi/flight_test_history/cosine.png) / [CSV](multi/flight_test_history/cosine.csv) | [图](multi/flight_test_history/arc.png) / [CSV](multi/flight_test_history/arc.csv) | [图](multi/flight_test_history/random.png) / [CSV](multi/flight_test_history/random.csv) | [图](multi/flight_test_history/multi_sine.png) / [CSV](multi/flight_test_history/multi_sine.csv) | [图](multi/flight_test_history/bspline.png) / [CSV](multi/flight_test_history/bspline.csv) |

### stable_current + accurate_delayed

| line | cosine | arc | random | multi-sine | B 样条 |
|---|---|---|---|---|---|
| [图](multi/stable_current_accurate/line.png) / [CSV](multi/stable_current_accurate/line.csv) | [图](multi/stable_current_accurate/cosine.png) / [CSV](multi/stable_current_accurate/cosine.csv) | [图](multi/stable_current_accurate/arc.png) / [CSV](multi/stable_current_accurate/arc.csv) | [图](multi/stable_current_accurate/random.png) / [CSV](multi/stable_current_accurate/random.csv) | [图](multi/stable_current_accurate/multi_sine.png) / [CSV](multi/stable_current_accurate/multi_sine.csv) | [图](multi/stable_current_accurate/bspline.png) / [CSV](multi/stable_current_accurate/bspline.csv) |
