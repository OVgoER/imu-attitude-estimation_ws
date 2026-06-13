# imu_attitude_estimation

ROS2 Jazzy + Gazebo 6-axis IMU attitude and inertial-state estimation benchmark.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Quick synthetic benchmark

```bash
ros2 run imu_attitude_estimation synthetic_benchmark --scenario fast_rotation --duration 12 --rate-hz 100
ros2 run imu_attitude_estimation synthetic_benchmark --scenario static_zero_drift --duration 12 --rate-hz 100
ros2 run imu_attitude_estimation synthetic_benchmark --scenario static_dynamic --duration 30 --rate-hz 100
ros2 run imu_attitude_estimation synthetic_benchmark --scenario trajectory --trajectory circle --duration 30 --rate-hz 100
ros2 run imu_attitude_estimation run_all_benchmarks --short
```

## ROS benchmark

```bash
ros2 launch imu_attitude_estimation benchmark.launch.py scenario:=fast_rotation duration:=35.0
ros2 launch imu_attitude_estimation static_zero_drift.launch.py
ros2 launch imu_attitude_estimation benchmark.launch.py scenario:=static_dynamic duration:=75.0
ros2 launch imu_attitude_estimation trajectory_tracking.launch.py trajectory:=figure8
```

Outputs are written to `results/` and `bags/`.
Gazebo GUI and the live Matplotlib window are enabled by default. Use
`use_gazebo:=false` or `live_plot:=false` only when running headless.

The ROS launch path uses the native C++ `imu_attitude_estimation_gtsam/gtsam_fgo_node`
for `/attitude/fgo`. The Python estimator keeps a SciPy fallback for environments
without GTSAM, but it is not published by default when the C++ backend is present.
