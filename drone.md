# Gazebo 无人机模型替换指南

## 📁 修改文件

`imu-attitude-estimation_ws/src/imu_attitude_estimation/worlds/imu_benchmark.sdf`

---

## ✅ 完整修改后的代码（直接复制替换）

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

        <!-- 机身 -->
        <visual name="body">
          <geometry><box><size>0.18 0.18 0.06</size></box></geometry>
          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
        </visual>

        <!-- 机臂 -->
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

        <!-- 旋翼 -->
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
      </link>
    </model>
  </world>
</sdf>
```

---

## 🔧 快速操作命令

```bash
# 1. 备份原文件
cp ~/imu-attitude-estimation_ws/src/imu_attitude_estimation/worlds/imu_benchmark.sdf ~/imu-attitude-estimation_ws/src/imu_attitude_estimation/worlds/imu_benchmark.sdf.bak

# 2. 替换文件内容（将上面的 XML 代码复制进去）
nano ~/imu-attitude-estimation_ws/src/imu_attitude_estimation/worlds/imu_benchmark.sdf

# 3. 重新编译
cd ~/imu-attitude-estimation_ws
colcon build --packages-select imu_attitude_estimation
source install/setup.bash

# 4. 运行
ros2 launch imu_attitude_estimation benchmark.launch.py \
  scenario:=fast_rotation trajectory:=yaw duration:=40.0 rate_hz:=200.0
```

---

## 📝 修改说明

| 项目 | 修改内容 |
|------|----------|
| 机身 | 从 0.18×0.12×0.05 改为 0.18×0.18×0.06 |
| 机臂 | 新增 4 个，尺寸 0.14×0.04×0.02 |
| 旋翼 | 新增 4 个，半径 0.07，厚度 0.02 |
| 碰撞体 | 调整为 0.18×0.18×0.06（与机身匹配） |
| IMU | 完全保持不变 |
