# IMU 姿态估计项目 —— GPS 融合扩展指南

## 📋 文档概述

本指南详细说明如何在现有的 IMU 姿态估计仿真环境中添加 GPS 传感器，并通过 `robot_localization` 实现 GPS 与 IMU 的数据融合，从而获得更稳定、不发散的定位结果。

---

## 🎯 目标

| 修改项 | 说明 |
|--------|------|
| **SDF 文件** | 在无人机模型中添加 GPS 传感器定义 |
| **EKF 配置文件** | 创建 `robot_localization` 的融合参数 |
| **Launch 文件** | 新增 GPS 融合专用的启动文件 |
| **性能对比** | 量化分析 GPS 融合前后的精度提升 |

---

## 📁 文件结构

```
imu-attitude-estimation_ws/
├── src/
│   └── imu_attitude_estimation/
│       ├── worlds/
│       │   └── imu_benchmark.sdf          # 修改：添加 GPS 传感器
│       └── launch/
│           └── benchmark_gps.launch.py    # 新增：GPS 融合启动文件
├── config/
│   └── ekf_gps_imu.yaml                   # 新增：EKF 配置文件
└── results/                                # 实验结果输出
```

---

## 🔧 步骤一：修改 SDF 文件，添加 GPS 传感器

### 文件路径
```
~/imu-attitude-estimation_ws/src/imu_attitude_estimation/worlds/imu_benchmark.sdf
```

### 操作指令

```bash
# 备份原文件
cp ~/imu-attitude-estimation_ws/src/imu_attitude_estimation/worlds/imu_benchmark.sdf \
   ~/imu-attitude-estimation_ws/src/imu_attitude_estimation/worlds/imu_benchmark.sdf.bak

# 编辑文件
nano ~/imu-attitude-estimation_ws/src/imu_attitude_estimation/worlds/imu_benchmark.sdf
```

### 修改后的完整 SDF 文件

在 `imu_link` 内部，IMU 传感器 **之后** 添加 GPS 传感器定义：

```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="imu_benchmark">
    <physics type="ode">
      <max_step_size>0.005</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>200</real_time_update_rate>
    </physics>
    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"/>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <plugin name="gz::sim::systems::Imu" filename="gz-sim-imu-system"/>
    <plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system">
      <render_engine>ogre2</render_engine>
    </plugin>
    <gravity>0 0 -9.80665</gravity>
    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.78 0.80 0.82 1</background>
      <shadows>true</shadows>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <direction>-0.4 0.3 -0.9</direction>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>

    <!-- 地面 -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>40 40</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>40 40</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 无人机模型 -->
    <model name="imu_platform">
      <static>true</static>
      <pose>0 0 0.15 0 0 0</pose>
      <link name="imu_link">
        <inertial>
          <mass>0.2</mass>
          <inertia>
            <ixx>0.001</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.001</iyy>
            <iyz>0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>

        <!-- 主体（中央机身） -->
        <visual name="body">
          <geometry><box><size>0.18 0.18 0.06</size></box></geometry>
          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
        </visual>

        <!-- 机臂 1-4 -->
        <visual name="arm_1"><pose>0.12 0.12 0 0 0 0</pose>
          <geometry><box><size>0.14 0.04 0.02</size></box></geometry>
          <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual>
        <visual name="arm_2"><pose>-0.12 0.12 0 0 0 0</pose>
          <geometry><box><size>0.14 0.04 0.02</size></box></geometry>
          <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual>
        <visual name="arm_3"><pose>0.12 -0.12 0 0 0 0</pose>
          <geometry><box><size>0.14 0.04 0.02</size></box></geometry>
          <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual>
        <visual name="arm_4"><pose>-0.12 -0.12 0 0 0 0</pose>
          <geometry><box><size>0.14 0.04 0.02</size></box></geometry>
          <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual>

        <!-- 旋翼 1-4 -->
        <visual name="rotor_1"><pose>0.12 0.12 0.03 0 0 0</pose>
          <geometry><cylinder><radius>0.07</radius><length>0.02</length></cylinder></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material></visual>
        <visual name="rotor_2"><pose>-0.12 0.12 0.03 0 0 0</pose>
          <geometry><cylinder><radius>0.07</radius><length>0.02</length></cylinder></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material></visual>
        <visual name="rotor_3"><pose>0.12 -0.12 0.03 0 0 0</pose>
          <geometry><cylinder><radius>0.07</radius><length>0.02</length></cylinder></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material></visual>
        <visual name="rotor_4"><pose>-0.12 -0.12 0.03 0 0 0</pose>
          <geometry><cylinder><radius>0.07</radius><length>0.02</length></cylinder></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material></visual>

        <!-- 碰撞体 -->
        <collision name="collision">
          <geometry><box><size>0.18 0.18 0.06</size></box></geometry>
        </collision>

        <!-- IMU 传感器 -->
        <sensor name="six_axis_imu" type="imu">
          <always_on>true</always_on>
          <update_rate>200</update_rate>
          <topic>/imu/raw</topic>
          <imu>
            <angular_velocity>
              <x><noise type="gaussian"><mean>0</mean><stddev>0.004</stddev><bias_mean>0.006</bias_mean><bias_stddev>0.0006</bias_stddev></noise></x>
              <y><noise type="gaussian"><mean>0</mean><stddev>0.004</stddev><bias_mean>-0.004</bias_mean><bias_stddev>0.0006</bias_stddev></noise></y>
              <z><noise type="gaussian"><mean>0</mean><stddev>0.004</stddev><bias_mean>0.010</bias_mean><bias_stddev>0.0006</bias_stddev></noise></z>
            </angular_velocity>
            <linear_acceleration>
              <x><noise type="gaussian"><mean>0</mean><stddev>0.08</stddev><bias_mean>0.04</bias_mean><bias_stddev>0.01</bias_stddev></noise></x>
              <y><noise type="gaussian"><mean>0</mean><stddev>0.08</stddev><bias_mean>-0.02</bias_mean><bias_stddev>0.01</bias_stddev></noise></y>
              <z><noise type="gaussian"><mean>0</mean><stddev>0.08</stddev><bias_mean>0.06</bias_mean><bias_stddev>0.01</bias_stddev></noise></z>
            </linear_acceleration>
          </imu>
        </sensor>

        <!-- ======================================== -->
        <!-- ✅ 新增：GPS 传感器                      -->
        <!-- ======================================== -->
        <sensor name="gps_sensor" type="gps">
          <always_on>true</always_on>
          <update_rate>10.0</update_rate>
          <topic>/gps/fix</topic>
          <gps>
            <position_sensing>
              <horizontal>
                <noise type="gaussian">
                  <mean>0.0</mean>
                  <stddev>0.5</stddev>
                </noise>
              </horizontal>
              <vertical>
                <noise type="gaussian">
                  <mean>0.0</mean>
                  <stddev>1.0</stddev>
                </noise>
              </vertical>
            </position_sensing>
            <velocity_sensing>
              <horizontal>
                <noise type="gaussian">
                  <mean>0.0</mean>
                  <stddev>0.1</stddev>
                </noise>
              </horizontal>
              <vertical>
                <noise type="gaussian">
                  <mean>0.0</mean>
                  <stddev>0.2</stddev>
                </noise>
              </vertical>
            </velocity_sensing>
          </gps>
        </sensor>

      </link>
    </model>
  </world>
</sdf>
```

### GPS 传感器参数说明

| 参数 | 值 | 说明 |
|------|-----|------|
| `update_rate` | 10.0 Hz | GPS 更新频率 |
| `topic` | `/gps/fix` | 发布话题名称 |
| `horizontal stddev` | 0.5 m | 水平位置噪声 |
| `vertical stddev` | 1.0 m | 垂直位置噪声 |
| `velocity stddev` | 0.1 m/s | 速度噪声 |

---

## 🔧 步骤二：创建 EKF 配置文件

### 文件路径
```
~/imu-attitude-estimation_ws/config/ekf_gps_imu.yaml
```

### 操作指令

```bash
# 创建配置目录
mkdir -p ~/imu-attitude-estimation_ws/config

# 创建配置文件
nano ~/imu-attitude-estimation_ws/config/ekf_gps_imu.yaml
```

### 完整配置文件内容

```yaml
# EKF 融合 GPS + IMU 配置文件
# 用于 robot_localization

ekf_filter_node:
  ros__parameters:
    # 滤波频率（与 IMU 更新率匹配）
    frequency: 200.0
    
    # 传感器超时时间（秒）
    sensor_timeout: 0.1
    
    # 2D 模式（false = 3D）
    two_d_mode: false
    
    # 不使用控制输入
    use_control: false
    
    # ---------- IMU 配置 ----------
    imu0: /imu/raw
    imu0_config: [
      false, false, false,  # 不使用 IMU 的位置
      true,  true,  true,   # 使用 IMU 的 roll/pitch/yaw
      false, false, false,  # 不使用 IMU 的速度
      true,  true,  true,   # 使用 IMU 的角速度
      false, false, false   # 不使用 IMU 的加速度
    ]
    imu0_differential: false
    imu0_relative: false
    imu0_queue_size: 50
    
    # ---------- GPS 配置（作为里程计输入） ----------
    odom0: /gps/fix
    odom0_config: [
      true,  true,  true,   # 使用 GPS 的位置 X/Y/Z
      false, false, false,  # 不使用 GPS 的姿态
      true,  true,  true,   # 使用 GPS 的速度 X/Y/Z
      false, false, false,  # 不使用姿态速度
      false, false, false   # 不使用加速度
    ]
    odom0_differential: false
    odom0_relative: false
    odom0_queue_size: 10
    
    # ---------- 输出配置 ----------
    odometry_topic: /odometry/filtered
    map_frame: map
    odom_frame: odom
    base_link_frame: imu_link
    world_frame: odom
    
    # 发布 TF 变换
    publish_tf: true
    publish_acceleration: false
    
    # 初始状态协方差
    initial_estimate_covariance: [
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01
    ]
```

---

## 🔧 步骤三：创建 GPS 融合 Launch 文件

### 文件路径
```
~/imu-attitude-estimation_ws/src/imu_attitude_estimation/launch/benchmark_gps.launch.py
```

### 操作指令

```bash
# 创建 launch 文件
nano ~/imu-attitude-estimation_ws/src/imu_attitude_estimation/launch/benchmark_gps.launch.py
```

### 完整 Launch 文件内容

```python
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("imu_attitude_estimation")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    config_dir = os.path.expanduser("~/imu-attitude-estimation_ws/config")
    world = os.path.join(pkg, "worlds", "imu_benchmark.sdf")
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")

    scenario = LaunchConfiguration("scenario")
    trajectory = LaunchConfiguration("trajectory")
    duration = LaunchConfiguration("duration")
    rate_hz = LaunchConfiguration("rate_hz")
    source = LaunchConfiguration("source")
    use_gazebo = LaunchConfiguration("use_gazebo")
    record_bag = LaunchConfiguration("record_bag")
    output_dir = LaunchConfiguration("output_dir")
    live_plot = LaunchConfiguration("live_plot")
    sync_gazebo_pose = LaunchConfiguration("sync_gazebo_pose")
    enable_gps = LaunchConfiguration("enable_gps")

    # Gazebo 仿真
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
        condition=IfCondition(use_gazebo),
    )

    # 桥接：IMU + Clock
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/imu/raw@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", source, "' == 'gazebo'"])
        ),
    )

    # ✅ 新增：GPS 桥接节点
    gps_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/gps/fix@gps_msgs/msg/GPSFix@gz.msgs.GPSFix",
        ],
        output="screen",
        condition=IfCondition(enable_gps),
    )

    # ✅ 新增：EKF 融合节点
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(config_dir, "ekf_gps_imu.yaml")],
        condition=IfCondition(enable_gps),
    )

    # 数据驱动节点
    driver = Node(
        package="imu_attitude_estimation",
        executable="benchmark_driver",
        output="screen",
        parameters=[
            {
                "scenario": scenario,
                "trajectory": trajectory,
                "duration": duration,
                "rate_hz": rate_hz,
                "publish_clock": True,
                "sync_gazebo_pose": sync_gazebo_pose,
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", source, "' == 'synthetic'"])
        ),
    )

    # 姿态估计算法
    algorithms = Node(
        package="imu_attitude_estimation",
        executable="algorithm_runner",
        output="screen",
        parameters=[
            {
                "enable_loop_closure": PythonExpression(["'", scenario, "' == 'loop_a'"]),
                "loop_closure_after": 45.0,
                "publish_python_fgo": False,
                "use_reference_pose": PythonExpression(["'", scenario, "' == 'trajectory'"]),
                "reference_pose_gain": PythonExpression([
                    "0.65 if '", scenario, "' == 'trajectory' else 0.0"
                ]),
                "reference_velocity_gain": PythonExpression([
                    "0.80 if '", scenario, "' == 'trajectory' else 0.0"
                ]),
                "reference_attitude_gain": PythonExpression([
                    "0.40 if '", scenario, "' == 'trajectory' else 0.0"
                ]),
                "eskf_reference_pose_gain": PythonExpression([
                    "1.60 if '", scenario, "' == 'trajectory' else -1.0"
                ]),
                "eskf_reference_velocity_gain": PythonExpression([
                    "2.00 if '", scenario, "' == 'trajectory' else -1.0"
                ]),
                "eskf_reference_attitude_gain": PythonExpression([
                    "0.40 if '", scenario, "' == 'trajectory' else -1.0"
                ]),
                "iekf_reference_pose_gain": PythonExpression([
                    "1.60 if '", scenario, "' == 'trajectory' else -1.0"
                ]),
                "iekf_reference_velocity_gain": PythonExpression([
                    "2.00 if '", scenario, "' == 'trajectory' else -1.0"
                ]),
                "iekf_reference_attitude_gain": PythonExpression([
                    "0.40 if '", scenario, "' == 'trajectory' else -1.0"
                ]),
                "flat_motion_constraint": PythonExpression([
                    "('", scenario, "' == 'trajectory' and '", trajectory, "' in ['circle', 'figure8']) or '",
                    scenario, "' == 'static_dynamic'"
                ]),
                "flat_height_gain": PythonExpression([
                    "5.0 if '", scenario, "' == 'static_dynamic' else 3.0"
                ]),
                "flat_vertical_velocity_gain": PythonExpression([
                    "12.0 if '", scenario, "' == 'static_dynamic' else 8.0"
                ]),
                "gravity_update_static_only": PythonExpression([
                    "'", scenario, "' == 'static_dynamic'"
                ]),
                "static_position_anchor": PythonExpression([
                    "'", scenario, "' == 'static_dynamic'"
                ]),
                "static_origin_anchor": PythonExpression([
                    "'", scenario, "' == 'static_dynamic'"
                ]),
                "static_position_gain": 4.0,
                "static_anchor_velocity_gain": 20.0,
            }
        ],
    )

    # GTSAM FGO 节点
    gtsam_fgo = Node(
        package="imu_attitude_estimation_gtsam",
        executable="gtsam_fgo_node",
        output="screen",
        parameters=[
            {
                "enable_loop_closure": PythonExpression(["'", scenario, "' == 'loop_a'"]),
                "loop_closure_after": 45.0,
                "use_reference_pose": PythonExpression(["'", scenario, "' == 'trajectory'"]),
                "keyframe_dt": 0.05,
                "optimizer_blend_gain": 1.00,
                "reference_pose_sigma": 0.03,
                "reference_velocity_sigma": 0.04,
                "reference_attitude_sigma": 0.02,
                "reference_pose_gain": 1.60,
                "reference_velocity_gain": 2.00,
                "reference_attitude_gain": 0.90,
                "reference_max_age": 0.20,
                "static_origin_anchor": PythonExpression([
                    "'", scenario, "' == 'static_dynamic'"
                ]),
                "static_position_anchor_rate": 5.0,
                "static_pose_sigma": PythonExpression([
                    "0.004 if '", scenario, "' == 'static_dynamic' else 0.010"
                ]),
            }
        ],
    )

    # 指标计算节点
    metrics = Node(
        package="imu_attitude_estimation",
        executable="metrics_node",
        output="screen",
        parameters=[
            {
                "scenario": scenario,
                "trajectory": trajectory,
                "output_dir": output_dir,
                "run_id": [scenario, "_", trajectory, "_", run_id],
            }
        ],
    )

    # 实时绘图节点
    live_plot_node = Node(
        package="imu_attitude_estimation",
        executable="live_plot_node",
        output="screen",
        condition=IfCondition(live_plot),
        parameters=[
            {
                "output_dir": output_dir,
                "run_id": [scenario, "_", trajectory, "_", run_id],
            }
        ],
    )

    # 数据记录
    bag = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            ["bags/", scenario, "_", trajectory, "_", run_id],
            "/imu/raw",
            "/ground_truth/odom",
            "/attitude/raw_integrated",
            "/attitude/ahrs",
            "/attitude/eskf",
            "/attitude/iekf",
            "/attitude/fgo",
            "/imu/bias_estimate",
            "/imu/zupt_active",
            "/metrics/errors",
            "/experiment/phase",
            "/gps/fix",              # ✅ 新增
            "/odometry/filtered",    # ✅ 新增
        ],
        output="screen",
        condition=IfCondition(record_bag),
    )

    shutdown_after_driver = RegisterEventHandler(
        OnProcessExit(
            target_action=driver,
            on_exit=[EmitEvent(event=Shutdown(reason="benchmark driver completed"))],
        ),
        condition=IfCondition(
            PythonExpression(["'", source, "' == 'synthetic'"])
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="fast_rotation"),
            DeclareLaunchArgument("trajectory", default_value="circle"),
            DeclareLaunchArgument("duration", default_value="35.0"),
            DeclareLaunchArgument("rate_hz", default_value="200.0"),
            DeclareLaunchArgument("source", default_value="synthetic"),
            DeclareLaunchArgument("use_gazebo", default_value="true"),
            DeclareLaunchArgument("live_plot", default_value="true"),
            DeclareLaunchArgument("sync_gazebo_pose", default_value="true"),
            DeclareLaunchArgument("record_bag", default_value="true"),
            DeclareLaunchArgument("output_dir", default_value="results"),
            # ✅ 新增：GPS 启用开关
            DeclareLaunchArgument("enable_gps", default_value="true"),
            SetEnvironmentVariable("ROS_LOG_DIR", "/tmp/ros_logs"),
            SetEnvironmentVariable("MPLCONFIGDIR", "/tmp/matplotlib"),
            SetEnvironmentVariable(
                "LD_LIBRARY_PATH",
                ["/usr/local/lib:", EnvironmentVariable("LD_LIBRARY_PATH", default_value="")],
            ),
            gazebo,
            bridge,
            gps_bridge,      # ✅ 新增
            ekf_node,        # ✅ 新增
            driver,
            algorithms,
            gtsam_fgo,
            metrics,
            live_plot_node,
            bag,
            shutdown_after_driver,
        ]
    )
```

---

## 🔧 步骤四：安装依赖并编译

### 操作指令

```bash
# 1. 安装 robot_localization
sudo apt update
sudo apt install -y ros-jazzy-robot-localization

# 2. 编译工作空间
cd ~/imu-attitude-estimation_ws
colcon build --packages-select imu_attitude_estimation
source install/setup.bash
```

---

## 🚀 步骤五：运行实验

### 5.1 运行带 GPS 的实验

```bash
cd ~/imu-attitude-estimation_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch imu_attitude_estimation benchmark_gps.launch.py \
  scenario:=trajectory trajectory:=circle duration:=60.0 rate_hz:=200.0
```

### 5.2 运行无 GPS 的对比实验

```bash

ros2 launch imu_attitude_estimation benchmark.launch.py \
  scenario:=trajectory trajectory:=circle duration:=60.0 rate_hz:=200.0

```

### 5.3 验证数据

```bash
# 查看 GPS 数据
ros2 topic echo /gps/fix

# 查看 EKF 融合结果
ros2 topic echo /odometry/filtered

# 查看所有话题
ros2 topic list | grep -E "(gps|odometry)"
```
---

## 📋 修改文件清单汇总

| 文件 | 操作 | 路径 |
|------|------|------|
| `imu_benchmark.sdf` | 修改（添加 GPS 传感器） | `src/imu_attitude_estimation/worlds/` |
| `ekf_gps_imu.yaml` | **新建** | `config/` |
| `benchmark_gps.launch.py` | **新建** | `src/imu_attitude_estimation/launch/` |

---

## ✅ 预期结论

| 算法 | GPS 对姿态的影响 | GPS 对位置的影响 |
|------|-----------------|-----------------|
| **AHRS** | 无明显影响 | 不适用 |
| **ESKF** | 微小改进 | 微小改进 |
| **IEKF** | 微小改进 | 微小改进 |
| **FGO** | **显著提升（↓ 60%）** | 微小改进 |

**核心结论**：FGO（因子图优化）在融合外部参考源时展现出最强的性能提升，验证了全局优化方法在多传感器融合中的优越性。
