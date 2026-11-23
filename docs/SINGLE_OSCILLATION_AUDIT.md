# 单机 60 Hz 飞行日志振荡复算

在仓库安装后运行 `python -m los_uav_interception.single_oscillation_audit trajectory.csv`。

输入沿用单机仿真 CSV 的列名，包括 `time_s`、三轴 `command_action_{north,east,down}` 和三轴 `actual_acceleration_{north,east,down}_mps2`。时间用秒，指令归一化至 ±1，加速度用 m/s²。若含 `interceptor_id`，整个文件须属于同一架飞机。时间戳需严格递增且采样间隔均匀，间隔容许约 2% 抖动。

程序从实际样本间隔判断频率，并沿用仓库的振荡判据。结果列出振荡标志、换向频率、频带能量占比、加速度峰峰值和 jerk RMS。默认同时计算每六点取一点后的 10 Hz 读数（针对 60 Hz 日志），用于对照采样口径。可用 `--reference-hz` 指定对照频率，频率需要与原样本构成整数抽取比例。抽取结果仅供诊断，可能漏掉高于对照频率奈奎斯特频率的变化。

日志如带有 `image_sample_time_s` 和 `applied_command_source_time_s`，各列均使用与 `time_s` 一致的时钟，分别表示当前观测的采样时刻和当前生效指令的生成时刻。程序会用记录时间差计算延迟中位数与 P95，单位为毫秒。
