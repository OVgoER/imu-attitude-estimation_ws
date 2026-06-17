## 首次命令
cd /home/zjy/imu-attitude-estimation_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
mkdir -p results bags

## 后续运行
cd /home/zjy/imu-attitude-estimation_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

实时曲线窗口默认读取本次实验正在写入的 `results/<scenario>_<trajectory>_<timestamp>.csv`，不再直接订阅算法 topic 重新计算误差。因此实时窗口和最终 `results/*_plots.png` 使用同一份 CSV 数据源；区别只是实时窗口按 `poll_period` 定时刷新。默认会显示从实验开始到当前时刻的完整曲线，时间轴随实验推进持续增长；如需恢复滑动窗口，可给 `live_plot_node` 设置正数 `max_points`。

## 指标口径

欧拉角误差只作为 `static_zero_drift` 静止零漂实验的主验收指标，用于观察 roll/pitch 漂移和姿态漂移率。快速旋转、动静切换、轨迹跟踪和 A -> 运动 -> A 回环实验中，roll/pitch/yaw 仅作为诊断参考；这些场景的优化和验收以整体 `att_err`、`pos_err`、recovery time、ATE/RPE 或回环误差为准，不为了压低单个欧拉角漂移去牺牲整体姿态误差曲线。

## 快速旋转
ros2 launch imu_attitude_estimation benchmark.launch.py \
  scenario:=fast_rotation trajectory:=yaw duration:=30.0 rate_hz:=200.0
可将 `trajectory:=yaw` 改为 `trajectory:=roll` 或 `trajectory:=pitch`，分别测试绕单一轴快速旋转。

快速旋转实验流程为：0-5s 静止初始化 -> 5-25s 以 360 deg/s 快速旋转 -> 25-35s 静止结束。`summary.yaml` 中 `summary_phase_filter: fast_*`，因此 RMSE、Peak 等核心指标只统计 5-25s 的快速旋转段，前后静止段只用于 bias 初始化和结束稳定性观察。
## 动静切换
ros2 launch imu_attitude_estimation benchmark.launch.py \
  scenario:=static_dynamic trajectory:=circle duration:=45.0 rate_hz:=200.0
流程为 45s 内持续循环：静止 5s -> 绕圈运动 5s -> 静止 5s。
## 轨迹跟踪：圆形
ros2 launch imu_attitude_estimation benchmark.launch.py \
  scenario:=trajectory trajectory:=circle duration:=60.0 rate_hz:=200.0
轨迹跟踪实验的 `results/*_plots.png` 不显示 roll/pitch/yaw 欧拉角误差，主要显示 `att err`、`pos err`、Ground Truth 与 AHRS/ESKF/IEKF/FGO 的三维轨迹对比。量化指标在 `results/*_summary.yaml` 中查看，轨迹实验额外包含 `ate_rmse`、`rpe_rmse`、`endpoint_position_error`。
## 轨迹跟踪：8 字形
ros2 launch imu_attitude_estimation benchmark.launch.py \
  scenario:=trajectory trajectory:=figure8 duration:=60.0 rate_hz:=200.0
## 轨迹跟踪：螺旋
ros2 launch imu_attitude_estimation benchmark.launch.py \
  scenario:=trajectory trajectory:=spiral duration:=60.0 rate_hz:=200.0
## A -> 运动 -> A 回环
ros2 launch imu_attitude_estimation benchmark.launch.py \
  scenario:=loop_a trajectory:=circle duration:=55.0 rate_hz:=200.0
当前 `loop_a` 使用三维闭合回环轨迹：5s 静止初始化，随后 40s 在三维空间内完成闭合运动，位置 `x/y/z` 与姿态 `roll/pitch/yaw` 都随时间变化，45s 后回到 A 点静止。`trajectory` 参数在该场景中仅保留为文件命名/兼容入口。
## 静止零漂
ros2 launch imu_attitude_estimation benchmark.launch.py \
  scenario:=static_zero_drift trajectory:=circle duration:=60.0 rate_hz:=200.0

请优化iekf，使att err和pos err的 RMSE和Peak降到最小，保证位置不漂、roll/pitch 稳、att_err 不出现 Peak 尖峰，请做到稳定最小而不是单次最小。yaw 作为漂移诊断单独看，不要为了压低 yaw_err 过度优化。

## gtsam安装：
cd gtsam-4.2.1
mkdir build
cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON

make -j$(nproc)

sudo make install

sudo ldconfig

## 快速旋转优化记录

### 优化目标

快速旋转主要用于验证算法在大角速度下的姿态保持能力。优化时采用以下原则：

- 优先压低 `att_err` 和 `pos_err` 的 RMSE/Peak。
- 保证位置不漂、roll/pitch 稳定，`att_err` 不出现明显尖峰。
- yaw 在 6 轴 IMU 下没有绝对观测源，只作为漂移诊断；不使用磁力计、视觉、GNSS 或 ground truth 伪测量去强行压低 `yaw_err`。
- 静止初始化段可用于估计 gyro/accel bias，但快速旋转段的 summary 只统计 `fast_*`，避免静止段把动态指标稀释掉。

### 共用实验与统计修改

相关代码：

- `src/imu_attitude_estimation/imu_attitude_estimation/trajectory.py`
- `src/imu_attitude_estimation/imu_attitude_estimation/metrics_node.py`
- `src/imu_attitude_estimation/launch/benchmark.launch.py`

优化内容：

- 将 `fast_rotation` 改为单轴快速旋转，可通过 `trajectory:=yaw|roll|pitch` 选择旋转轴。
- 快速段角速度固定为 `360 deg/s = 2*pi rad/s`，持续 20s。
- 恢复 5s 静止初始化和 10s 结束静止段，用于 bias 估计、静止约束和曲线健康检查。
- `metrics_node` 对 `fast_rotation` 只累计 `phase.startswith("fast_")` 的样本，所以 `summary.yaml` 只反映快速旋转段性能。

原理说明：

如果直接从 0s 开始高速 yaw 旋转，gyro bias 没有静止数据可估计，`yaw_err` 会立刻积分进 `att_err`，不同算法会被同一个不可观测 yaw 漂移主导，差异反而被掩盖。先静止初始化、再只统计旋转段，可以更公平地比较“已完成初始化后”的动态跟踪能力。

### AHRS 优化

相关代码：

- `src/imu_attitude_estimation/imu_attitude_estimation/estimators.py`
- `src/imu_attitude_estimation/launch/benchmark.launch.py`

快速旋转参数：

| 参数 | fast_rotation 值 | 作用 |
| --- | ---: | --- |
| `ahrs_kp` | `1.8` | 增强重力方向对 roll/pitch 的比例修正 |
| `ahrs_ki` | `0.01` | 降低积分修正，避免把动态噪声积进 gyro bias |
| `static_bias_gain` | `0.02` | 静止初始化时平滑估计 gyro bias |
| `ahrs_velocity_damping` | `0.3` | 静止时抑制速度残差 |
| `ahrs_stationary_translation_constraint` | `true` | 快速旋转场景下启用位置锚定 |
| `ahrs_stationary_position_gain` | `10.0` | 静止/原地旋转时位置回拉强度 |
| `ahrs_stationary_velocity_gain` | `16.0` | 静止/原地旋转时速度回零强度 |

优化内容：

- 重力误差改为在 IMU body frame 中计算：先用当前姿态把世界系重力反算到 body frame，再与加速度计测得的重力方向做叉乘。
- 静止时用较小 `static_bias_gain` 估计 gyro bias，避免单帧噪声导致 bias 抖动。
- 快速旋转下提高 `kp`、降低 `ki`：roll/pitch 需要快速收敛，但 yaw 不可观，不应靠积分项硬拉。
- 启用 stationary translation constraint，将原地旋转的位置和速度限制在静止锚点附近，避免加速度 bias 被二次积分成位置漂移。

原理说明：

AHRS 的优势是轻量、响应快；缺点是没有完整状态协方差，动态加速度和 gyro bias 容易耦合。快速旋转时，重力只能约束 roll/pitch，不能约束 yaw。因此优化重点是让 roll/pitch 修正方向正确、bias 估计平滑、位置不要因加速度噪声积分漂移，而不是强行压低 yaw。

### ESKF 优化

相关代码：

- `src/imu_attitude_estimation/imu_attitude_estimation/estimators.py`
- `src/imu_attitude_estimation/imu_attitude_estimation/estimator_base.py`
- `src/imu_attitude_estimation/launch/benchmark.launch.py`

快速旋转参数：

| 参数 | fast_rotation 值 | 作用 |
| --- | ---: | --- |
| `eskf_gravity_gain` | `4.0` | roll/pitch 重力观测更新强度 |
| `eskf_zupt_gain` | `45.0` | 静止时速度归零强度 |
| `eskf_bias_gain` | `1.5` | 静止时 gyro/accel bias 更新速度 |
| `eskf_stationary_translation_constraint` | `true` | 原地旋转位置锚定 |
| `eskf_stationary_position_gain` | `10.0` | 位置锚点回拉强度 |
| `eskf_stationary_velocity_gain` | `16.0` | 速度回零强度 |

优化内容：

- 预测阶段继续使用 IMU 积分，更新阶段使用重力方向修正 roll/pitch，并显式把 yaw 误差通道置零，避免把不可观 yaw 当作重力可观测量修正。
- 重力观测加入低通和加速度模长置信度：`|accel|-g` 偏差越大，重力修正越弱。
- 所有更新增益改为 `1 - exp(-rate * dt)` 形式，保证 100 Hz/200 Hz 下等效时间常数一致。
- 静止时执行 ZUPT/ZARU：速度回零、gyro bias 向静止角速度收敛、accel bias 按当前姿态下的重力投影估计。
- 静止/原地旋转时启用 translation anchor，把位置和速度限制在起始锚点附近。

原理说明：

ESKF 的优势是误差状态建模清晰，适合在高速旋转时分开处理预测误差、重力观测和 bias 更新。快速 yaw 中，yaw 主要由 gyro bias 残差积分形成；没有外部航向源时，健康曲线应是小幅缓慢漂移，而不是被伪观测强行拉回 0。ESKF 的优化重点是保护可观的 roll/pitch 和位置通道，同时让不可观 yaw 自然暴露为漂移诊断。

### IEKF 优化

相关代码：

- `src/imu_attitude_estimation/imu_attitude_estimation/estimators.py`
- `src/imu_attitude_estimation/launch/benchmark.launch.py`

快速旋转参数：

| 参数 | fast_rotation 值 | 作用 |
| --- | ---: | --- |
| `iekf_gravity_gain` | `4.0` | roll/pitch 重力修正强度 |
| `iekf_zupt_gain` | `45.0` | 静止速度归零强度 |
| `iekf_bias_gain` | `1.5` | 静止 bias 更新速度 |
| `iekf_gravity_step_cap` | `0.16` | 限制单步重力修正，防止尖峰 |
| `iekf_stationary_translation_constraint` | `true` | 原地旋转位置锚定 |
| `iekf_stationary_position_gain` | `10.0` | 位置锚点回拉强度 |
| `iekf_stationary_velocity_gain` | `16.0` | 速度回零强度 |

优化内容：

- 预测阶段使用 midpoint 姿态计算世界系加速度，降低快速旋转时“先姿态后加速度”离散化误差。
- 重力更新沿用 ESKF 的低通、置信度和 yaw 不可观处理，但增加 `iekf_gravity_step_cap=0.16` 限制单步姿态注入。
- 协方差增长使用较保守的 `q_scale=0.82`，避免高频噪声在快速旋转段被过度放大。
- 静止 ZUPT/ZARU、bias 更新和 stationary translation constraint 与 ESKF 保持一致，保证位置不漂。

原理说明：

IEKF 相比 ESKF 更强调在当前流形状态附近做误差注入，快速旋转时 midpoint 积分可以降低离散化误差；但如果重力更新过强，也可能把加速度噪声注入 roll/pitch 形成尖峰。因此 IEKF 的关键是“更准确的预测 + 更保守的单步修正”，目标是稳定最小，而不是单次最小。

### FGO 优化

相关代码：

- `src/imu_attitude_estimation_gtsam/src/gtsam_fgo_node.cpp`
- `src/imu_attitude_estimation/launch/benchmark.launch.py`

通用 FGO 框架参数：

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| `keyframe_dt` | `0.10` | 常规关键帧间隔，避免所有场景都过密建图 |
| `min_keyframe_dt` | `0.035` | 快速运动时的最小关键帧间隔 |
| `max_keyframe_rotation_rad` | `0.25` | 按角速度自适应缩短关键帧间隔，避免大角速度下单个 IMU 因子跨度过大 |
| `window_duration_sec` | `5.0` | 普通滑窗时间长度 |
| `loop_window_duration_sec` | `65.0` | 回环场景保留完整历史，支持首末位姿一致性 |
| `optimizer_blend_gain` | `0.78` | 优化结果与在线预积分预测小步融合，避免高频锯齿 |
| `optimizer_*_step_limit` | `att=0.020, pos=0.060, vel=0.150` | 限制单次优化写回步长，抑制 Peak 尖峰 |
| `bias_prior_*_sigma` | `accel=0.20, gyro=0.030` | 初始 bias 先验与随机游走解耦，避免过紧 bias 先验锁死漂移 |
| `translation_stationary` | `true` | 静止或零平移旋转时加入位置/速度约束 |
| `use_reference_pose` | `trajectory` 场景启用 | 轨迹跟踪场景使用外部参考位姿作为通用观测 |
| `enable_loop_closure` | `loop_a` 场景启用 | A -> 运动 -> A 回环使用首末位姿一致性约束 |

优化内容：

- 从“按场景调窗口大小”改为“按运动状态建约束”：IMU 预积分始终从固定 keyframe 状态预测，关键帧间隔按角速度自适应，普通轨迹、快速旋转和三维回环共用同一套传播逻辑。
- 因子图同时支持 IMU 因子、bias between 因子、静止 ZUPT/ZARU、translation prior、参考位姿 prior 和 loop closure prior。不同实验只决定哪些观测真实存在，不再为了某条轨迹硬编码特殊解。
- 静止锚点有 hysteresis，避免动静边界抖动；闭环激活后静止锚点切换为 A 点完整位姿，避免“当前错误姿态锚点”和“回环位姿约束”互相打架。
- 在线发布端使用优化结果小步融合和 step limit，并在外部/回环约束修正后同步最新 keyframe 与传播基准，防止下一帧又被旧预积分链路拉回。
- 对原地快速旋转使用零平移检测和 translation stationary 约束来压住位置漂移；对三维轨迹和回环不假设平面或固定高度。
- `yaw_err` 仍只作为漂移诊断。快速旋转中不使用外部 yaw 参考；只有回环场景在物理上已经回到 A 点时，才使用完整 A 点位姿作为闭环约束。

原理说明：

通用 FGO 的核心不是把参数调到某个场景最好，而是把“哪些状态可观”表达成因子。快速旋转主要依靠 IMU 预积分、bias 建模和零平移约束；动静切换依靠静止 ZUPT/ZARU 和 bias 更新；轨迹跟踪依靠外部参考位姿；回环依靠首末位姿一致性。这样 FGO 的优势体现在全局一致性和多约束融合上，同时通过在线融合和步长限制避免滑窗优化常见的高频锯齿。

### 当前快速旋转验收结果

验证命令：

```bash
ros2 launch imu_attitude_estimation benchmark.launch.py \
  scenario:=fast_rotation trajectory:=yaw duration:=35.0 rate_hz:=200.0 \
  use_gazebo:=false live_plot:=false record_bag:=false sync_gazebo_pose:=false
```

结果文件：

- `results/fast_rotation_yaw_20260616_034953.csv`
- `results/fast_rotation_yaw_20260616_034953_summary.yaml`
- `results/fast_rotation_yaw_20260616_034953_plots.png`

核心指标只统计 5-25s 的 `fast_yaw` 段：

| algorithm | position RMSE [m] | attitude RMSE [rad] | position Peak [m] | attitude Peak [rad] |
| --- | ---: | ---: | ---: | ---: |
| raw integrated | 35.510806 | 0.163658 | 75.619568 | 0.252322 |
| AHRS | 0.000251 | 0.009106 | 0.000410 | 0.014972 |
| ESKF | 0.000179 | 0.005678 | 0.000313 | 0.008802 |
| IEKF | 0.000178 | 0.005678 | 0.000313 | 0.008802 |
| FGO | 0.000211 | 0.004136 | 0.000475 | 0.006518 |

结论：

- FGO 当前姿态误差最小，`attitude RMSE` 和 `attitude Peak` 最低。
- IEKF/ESKF 当前位置误差最小，两者几乎一致。
- AHRS 明显优于原始积分，但作为轻量基线，姿态精度低于 ESKF/IEKF/FGO。
- yaw 误差仍单独作为漂移诊断查看，不作为强行调参压低的主目标。

## 静止零漂优化记录

### 问题现象

静止零漂测试中曾出现以下现象：

- AHRS 的 yaw 漂移接近 0，静止表现最好。
- ESKF/IEKF 的 roll、pitch 曲线有锯齿。
- FGO 的 yaw 有锯齿，长时间静止时 roll、pitch 也会出现阶跃。
- FGO 在静止零漂场景下不一定优于 ESKF/IEKF，和预期排序不一致。

这些现象的主要原因不是算法理论失效，而是静止约束、bias 校准和 FGO 在线发布方式不够稳定。

### 原理说明

6 轴 IMU 没有磁力计、视觉、GNSS 等绝对参考源，因此 yaw 在全局意义上不可完全观测。静止状态下可以利用以下伪测量抑制漂移：

- ZUPT：静止时速度应收敛到 0。
- ZARU：静止时角速度应接近 0，可用于估计 gyro bias。
- Gravity update：加速度计测到的重力方向可修正 roll/pitch，但不能提供绝对 yaw。
- Heading hold：进入静止段时锁定当前 yaw，静止期间抑制 yaw 随 gyro bias 漂移。

因此，静止零漂验收中 AHRS 的 yaw 很小是合理的，因为 AHRS 使用了更直接的静止 bias/ZARU 抑制。FGO 如果只加入速度 ZUPT，而没有静止姿态/yaw 伪测量，yaw 仍会漂移；如果静止锚点频繁重置，还会产生锯齿。

### 修改内容

Python 估计器：`src/imu_attitude_estimation/imu_attitude_estimation/estimators.py`

- AHRS 的重力误差改为在 body frame 中计算。旧方案把估计重力方向和测量加速度放在不一致的坐标系里做叉乘，修正方向在部分姿态下会偏甚至反向，表现为 roll/pitch 收敛慢、局部抖动，并间接影响 yaw。新方案先把世界系重力用当前姿态反算到 IMU body frame，再和加速度计测得的重力方向比较，使姿态误差反馈和陀螺积分处于同一坐标系。
- AHRS/ESKF/IEKF 的 accel bias 校准不再假设 IMU 一定水平。旧方案静止时直接用 `sample.accel - [0, 0, g]` 估计加速度零偏，等价于默认 IMU roll/pitch 为 0；当 IMU 带初始倾角或姿态估计有小误差时，会把真实重力投影误认为 accel bias，导致 roll/pitch 漂移和速度残差。新方案使用当前姿态将世界系重力反算到 body frame 后再估计 bias，使静止零偏校准与实际姿态一致。
- ESKF/IEKF 的重力修正、ZUPT、bias 更新、yaw anchor 改为按 `dt` 计算连续时间增益。旧方案使用每帧固定增益，同一组参数在 100 Hz 和 200 Hz 下等效强度不同，频率越高越容易过度修正噪声，曲线上会出现锯齿。新方案使用 `1 - exp(-rate * dt)` 形式计算每步增益，让修正强度由时间常数决定，而不是由帧率决定，因此静止段更平滑，换采样率后也更稳定。
- ESKF/IEKF 增加重力加速度低通和加速度模长置信度。旧方案每帧直接把瞬时加速度当成重力方向，白噪声或微小线加速度会直接进入 roll/pitch 更新，造成 pitch/roll 高频锯齿。新方案先对加速度做低通，并根据 `|accel|-g` 的偏差降低重力观测权重；当加速度更像真实重力时才强修正，动态或噪声较大时自动减弱修正。
- 静止段增加 yaw anchor。旧方案只靠 gyro bias/ZARU 抑制 yaw，静止检测抖动或 bias 估计未完全收敛时仍会有 yaw slope。新方案在进入静止段时记录当前 yaw，静止期间用小增益把 yaw 拉回该锚点，同时继续估计 gyro bias；这不引入绝对航向源，只是利用“静止期间航向应保持不变”的伪测量抑制漂移。

C++ GTSAM FGO：`src/imu_attitude_estimation_gtsam/src/gtsam_fgo_node.cpp`

- IMU 预积分预测改为从固定 keyframe 状态预测到当前状态。旧方案在每个 IMU 回调中用不断变化的 `current_pose/current_velocity` 再叠加同一段累计预积分，相当于重复使用已经积过的 IMU 增量，容易造成 FGO 在线输出锯齿和姿态/速度跳变。新方案保存 `keyframe_pose/keyframe_velocity/keyframe_bias`，当前帧始终由固定 keyframe 加上当前预积分预测得到，关键帧更新后再重置预积分。
- 图优化后把滑窗内所有 keyframe 状态写回。旧方案只写回最后一个 keyframe，滑窗前面的节点仍是旧线性化状态；下一次优化会在“部分新、部分旧”的窗口上重新求解，关键帧处容易出现不连续。新方案把优化后的 pose、velocity、bias 全部写回滑窗，使下一轮图优化从一致状态开始。
- 静止段加入 ZUPT 速度先验和静止 pose/yaw anchor 先验。旧 FGO 只有 IMU 因子和速度约束，6 轴 IMU 下 yaw 不可观，roll/pitch 也容易被噪声加速度和 bias 耦合拖动，所以静止时仍会漂移。新方案在静止关键帧加入速度为 0 的先验，并加入静止姿态/位置锚点；roll/pitch 由重力方向约束，yaw 由 heading hold 约束，从因子图层面压低静止漂移。
- 增加静止进入/退出滞回。旧方案每帧直接使用瞬时静止判定，噪声会让状态在静止/非静止之间来回切换，导致静止锚点反复捕获，表现为 roll/pitch/yaw 阶跃和锯齿。新方案要求连续多帧满足静止条件才进入静止模式，连续多帧不满足才退出，避免单帧噪声改变约束状态。
- 静止锚点只在进入静止模式时捕获一次。旧方案可能在长时间静止中多次刷新锚点，而新锚点若来自带噪声的单帧加速度，就会把噪声固化为新的姿态先验。新方案进入静止后保持同一锚点，静止期间只围绕该锚点收敛，不再被单帧噪声重置。
- 实时发布端对优化结果做小步融合。旧方案在每个 keyframe 优化完成后直接跳到优化结果，曲线会在优化时刻出现尖峰；这不是 IMU 真实运动，而是批量优化的离散更新造成的显示跳变。新方案将预积分在线预测与优化结果按小增益融合，让实时曲线连续，同时保留 FGO 对 bias、速度和姿态的长期校正能力。
- 静止期间对 roll、pitch、yaw 都小步回拉到静止锚点。旧方案主要压速度和 yaw，roll/pitch 仍可能被加速度噪声、bias 或预积分残差拉出锯齿。新方案用低通后的重力方向确定 roll/pitch 锚点，用 heading hold 确定 yaw 锚点，并在发布状态中连续回拉；因此静止零漂里 roll/pitch 大锯齿被压掉，yaw 漂移也被限制在小范围内。
