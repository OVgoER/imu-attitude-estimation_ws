快速旋转
ros2 launch imu_attitude_estimation benchmark.launch.py \
  scenario:=fast_rotation duration:=35.0 rate_hz:=200.0
动静切换
ros2 launch imu_attitude_estimation benchmark.launch.py \
  scenario:=static_dynamic duration:=75.0 rate_hz:=200.0
轨迹跟踪：圆形
ros2 launch imu_attitude_estimation benchmark.launch.py \
  scenario:=trajectory trajectory:=circle duration:=60.0 rate_hz:=200.0
轨迹跟踪：8 字形
ros2 launch imu_attitude_estimation benchmark.launch.py \
  scenario:=trajectory trajectory:=figure8 duration:=60.0 rate_hz:=200.0
轨迹跟踪：螺旋
ros2 launch imu_attitude_estimation benchmark.launch.py \
  scenario:=trajectory trajectory:=spiral duration:=60.0 rate_hz:=200.0
A -> 运动 -> A 回环
ros2 launch imu_attitude_estimation benchmark.launch.py \
  scenario:=loop_a trajectory:=circle duration:=55.0 rate_hz:=200.0
静止零漂
ros2 launch imu_attitude_estimation benchmark.launch.py \
  scenario:=static_zero_drift trajectory:=circle duration:=60.0 rate_hz:=200.0
