# ROS2 + Gazebo 6轴IMU状态优化仿真实施计划

## Summary

- 实施前先创建 `docs/imu_attitude_estimation_plan.md`，写入本计划全文。
- 创建 ROS2 Jazzy 包，基于 Gazebo Sim 搭建 6轴 IMU 仿真。
- 实现 AHRS、ESKF、IEKF、FGO+SE(3)+IMU预积分四种算法。
- 输出原始 IMU、Ground Truth、四种估计结果、误差指标、CSV、rosbag 和对比图。

## Key Changes

- 新建 `docs/`，添加计划文档 `imu_attitude_estimation_plan.md`。
- 新建主包 `imu_attitude_estimation`，包含 Gazebo 世界、IMU 模型、launch、算法节点、实验脚本和评估工具。
- 安装并使用 GTSAM 作为 FGO 后端；若缺少依赖，优先安装 `ros-jazzy-gtsam`。
- 统一输出目录：`results/`、`bags/`、`docs/`。

## Algorithms

- AHRS：重力辅助 roll/pitch 修正，gyro bias 校准，ZARU 抑制静止 yaw 漂移。
- ESKF：状态包含 pose、velocity、orientation、gyro bias、accel bias，加入 ZUPT/ZARU。
- IEKF：基于 SE(3)/SO(3) 误差定义，增强大姿态变化下稳定性。
- FGO：使用 GTSAM IMU 预积分、bias 因子、静止约束、闭环伪测量因子。

## Experiments

- 快速旋转：roll/pitch/yaw 单轴与组合快速旋转，评价姿态误差。
- 动静切换：静止和运动交替循环，评价静止段速度是否收敛到 0、yaw 是否恢复稳定。
- 轨迹跟踪：圆形、8字形、螺旋轨迹，对比 Ground Truth 和四种估计轨迹。
- A -> 任意运动 -> 回到 A：用于验证闭环后终点位姿误差。
- 每个实验均生成 CSV、rosbag、summary YAML 和误差图。

## Test Plan

- `colcon build` 构建通过。
- 三个 benchmark launch 均可启动并生成完整输出。
- 静止仿真验收：AHRS/ESKF/IEKF/FGO 的 roll/pitch 漂移明显小于原始积分，yaw 漂移被 bias/ZARU 显著压低。
- A -> 运动 -> A 验收：FGO 加闭环伪测量后终点位姿误差最小；ESKF/IEKF 不使用闭环时仍保持合理漂移。
- 快速旋转验证姿态无跳变，统计姿态 RMSE 和峰值误差。
- 动静切换验证静止段速度收敛、bias 收敛、yaw 漂移斜率降低。
- 轨迹实验验证 ATE、RPE、终点闭环误差和姿态误差。

## Assumptions

- 6轴 IMU 无绝对航向源，yaw 和全局位置不可完全观测。
- Ground Truth 只用于评估，不进入 AHRS/ESKF/IEKF。
- FGO 可在回到 A 点实验中使用闭环伪测量约束。
- 实施阶段必须先写入计划文档，再修改代码。
