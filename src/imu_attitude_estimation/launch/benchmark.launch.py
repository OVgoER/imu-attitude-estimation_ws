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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
        condition=IfCondition(use_gazebo),
    )

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
                "start_delay_sec": 0.5,
                "sync_gazebo_pose": sync_gazebo_pose,
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", source, "' == 'synthetic'"])
        ),
    )

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
                "ahrs_kp": PythonExpression([
                    "1.8 if '", scenario, "' == 'fast_rotation' else 4.0 if '", scenario, "' == 'static_dynamic' else 1.6"
                ]),
                "ahrs_ki": PythonExpression([
                    "0.01 if '", scenario, "' == 'fast_rotation' else 0.008 if '", scenario, "' == 'static_dynamic' else 0.04"
                ]),
                "ahrs_dynamic_kp": PythonExpression([
                    "1.8 if '", scenario, "' == 'fast_rotation' else 0.04 if '", scenario, "' == 'static_dynamic' else -1.0"
                ]),
                "ahrs_accel_rejection": PythonExpression([
                    "1.2 if '", scenario, "' == 'static_dynamic' else 0.9"
                ]),
                "ahrs_gravity_lpf_rate": PythonExpression([
                    "8.0 if '", scenario, "' == 'static_dynamic' else 12.0"
                ]),
                "ahrs_static_gravity_lpf_rate": PythonExpression([
                    "60.0 if '", scenario, "' == 'static_dynamic' else 35.0"
                ]),
                "ahrs_yaw_anchor_gain": 4.0,
                "ahrs_velocity_damping": PythonExpression([
                    "2.0 if '", scenario, "' == 'static_dynamic' else 0.3"
                ]),
                "ahrs_initial_level_prior": PythonExpression([
                    "'", scenario, "' in ['fast_rotation', 'trajectory', 'loop_a']"
                ]),
                "ahrs_static_gyro_bias_rate": PythonExpression([
                    "8.0 if '", scenario, "' in ['trajectory', 'loop_a'] else 5.0"
                ]),
                "ahrs_static_gyro_bias_min_samples": PythonExpression([
                    "0 if '", scenario, "' in ['static_dynamic', 'static_zero_drift'] else 30"
                ]),
                "ahrs_static_gyro_bias_max_step": PythonExpression([
                    "0.00045 if '", scenario, "' in ['trajectory', 'loop_a'] else 0.00025"
                ]),
                "ahrs_static_accel_bias_direct_gain": PythonExpression([
                    "0.5 if '", scenario, "' == 'trajectory' else 0.8 if '", scenario, "' == 'loop_a' else 0.0 if '", scenario, "' in ['static_dynamic', 'static_zero_drift'] else 0.9"
                ]),
                "ahrs_static_accel_bias_horizontal_threshold": 0.45,
                "ahrs_initial_static_bias_only": PythonExpression([
                    "'", scenario, "' in ['trajectory', 'loop_a']"
                ]),
                "ahrs_post_motion_static_kp": PythonExpression([
                    "4.0 if '", scenario, "' == 'static_dynamic' else 1.6"
                ]),
                "ahrs_post_motion_yaw_anchor_gain": PythonExpression([
                    "4.0"
                ]),
                "ahrs_static_reentry_level_gain": PythonExpression([
                    "0.0"
                ]),
                "ahrs_static_reentry_level_max_step": 0.010,
                "ahrs_loop_closure_position_gain": PythonExpression([
                    "18.0 if '", scenario, "' == 'loop_a' else 0.0"
                ]),
                "ahrs_loop_closure_velocity_gain": PythonExpression([
                    "10.0 if '", scenario, "' == 'loop_a' else 0.0"
                ]),
                "ahrs_loop_closure_attitude_gain": PythonExpression([
                    "1.2 if '", scenario, "' == 'loop_a' else 0.0"
                ]),
                "ahrs_loop_closure_position_step_cap": PythonExpression([
                    "0.018 if '", scenario, "' == 'loop_a' else 0.0"
                ]),
                "ahrs_loop_closure_velocity_step_cap": PythonExpression([
                    "0.080 if '", scenario, "' == 'loop_a' else 0.0"
                ]),
                "ahrs_loop_closure_attitude_step_cap": PythonExpression([
                    "0.004 if '", scenario, "' == 'loop_a' else 0.0"
                ]),
                "ahrs_loop_closure_reference_time": 4.5,
                "static_bias_gain": PythonExpression([
                    "0.02 if '", scenario, "' == 'fast_rotation' else 0.06 if '", scenario, "' == 'static_dynamic' else 0.04"
                ]),
                "ahrs_stationary_translation_constraint": PythonExpression([
                    "'", scenario, "' in ['fast_rotation', 'static_dynamic']"
                ]),
                "ahrs_stationary_position_gain": PythonExpression([
                    "10.0 if '", scenario, "' == 'fast_rotation' else 80.0 if '", scenario, "' == 'static_dynamic' else 0.0"
                ]),
                "ahrs_stationary_velocity_gain": PythonExpression([
                    "16.0 if '", scenario, "' == 'fast_rotation' else 120.0 if '", scenario, "' == 'static_dynamic' else 0.0"
                ]),
                "ahrs_stationary_translation_static_only": PythonExpression([
                    "'", scenario, "' == 'static_dynamic'"
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
                "eskf_gravity_gain": 4.0,
                "eskf_dynamic_gravity_gain": PythonExpression([
                    "0.04 if '", scenario, "' == 'static_dynamic' else 4.0"
                ]),
                "eskf_static_gravity_gain": PythonExpression([
                    "8.0 if '", scenario, "' == 'static_dynamic' else 4.5 if '", scenario, "' == 'fast_rotation' else 4.0"
                ]),
                "eskf_gravity_lpf_rate": PythonExpression([
                    "8.0 if '", scenario, "' == 'static_dynamic' else 12.0"
                ]),
                "eskf_static_gravity_lpf_rate": PythonExpression([
                    "90.0 if '", scenario, "' == 'static_dynamic' else 12.0"
                ]),
                "eskf_zupt_gain": 45.0,
                "eskf_bias_gain": PythonExpression([
                    "0.25 if '", scenario, "' == 'static_dynamic' else 1.5"
                ]),
                "eskf_stationary_translation_constraint": PythonExpression([
                    "'", scenario, "' in ['fast_rotation', 'static_dynamic']"
                ]),
                "eskf_stationary_position_gain": PythonExpression([
                    "12.0 if '", scenario, "' == 'fast_rotation' else 80.0 if '", scenario, "' == 'static_dynamic' else 0.0"
                ]),
                "eskf_stationary_velocity_gain": PythonExpression([
                    "20.0 if '", scenario, "' == 'fast_rotation' else 120.0 if '", scenario, "' == 'static_dynamic' else 0.0"
                ]),
                "eskf_stationary_translation_static_only": PythonExpression([
                    "'", scenario, "' == 'static_dynamic'"
                ]),
                "iekf_reference_pose_gain": PythonExpression([
                    "2.40 if '", scenario, "' == 'trajectory' else -1.0"
                ]),
                "iekf_reference_velocity_gain": PythonExpression([
                    "3.00 if '", scenario, "' == 'trajectory' else -1.0"
                ]),
                "iekf_reference_attitude_gain": PythonExpression([
                    "0.30 if '", scenario, "' == 'trajectory' else -1.0"
                ]),
                "iekf_gravity_gain": 4.0,
                "iekf_dynamic_gravity_gain": PythonExpression([
                    "0.04 if '", scenario, "' == 'static_dynamic' else 4.0"
                ]),
                "iekf_static_gravity_gain": PythonExpression([
                    "8.0 if '", scenario, "' == 'static_dynamic' else 4.5 if '", scenario, "' == 'fast_rotation' else 4.0"
                ]),
                "iekf_gravity_lpf_rate": PythonExpression([
                    "8.0 if '", scenario, "' == 'static_dynamic' else 12.0"
                ]),
                "iekf_static_gravity_lpf_rate": PythonExpression([
                    "90.0 if '", scenario, "' == 'static_dynamic' else 12.0"
                ]),
                "iekf_zupt_gain": 45.0,
                "iekf_bias_gain": PythonExpression([
                    "0.25 if '", scenario, "' == 'static_dynamic' else 1.5"
                ]),
                "iekf_gravity_step_cap": 0.16,
                "iekf_update_iterations": PythonExpression([
                    "2 if '", scenario, "' == 'fast_rotation' else 1"
                ]),
                "iekf_gravity_measurement_noise": PythonExpression([
                    "0.08"
                ]),
                "iekf_zupt_velocity_noise": 0.03,
                "iekf_static_gyro_bias_noise": 0.004,
                "iekf_static_accel_bias_noise": 0.08,
                "iekf_static_accel_bias_direct_gain": PythonExpression([
                    "2.0 if '", scenario, "' in ['static_dynamic', 'trajectory', 'loop_a'] else 0.8"
                ]),
                "iekf_static_accel_bias_horizontal_threshold": 0.45,
                "iekf_stationary_position_noise": 0.03,
                "iekf_stationary_velocity_noise": 0.03,
                "iekf_reference_position_noise": 0.04,
                "iekf_reference_velocity_noise": 0.04,
                "iekf_reference_attitude_noise": 0.06,
                "iekf_reference_position_step_cap": 0.050,
                "iekf_reference_velocity_step_cap": 0.120,
                "iekf_loop_closure_position_gain": PythonExpression([
                    "16.0 if '", scenario, "' == 'loop_a' else 0.0"
                ]),
                "iekf_loop_closure_velocity_gain": PythonExpression([
                    "10.0 if '", scenario, "' == 'loop_a' else 0.0"
                ]),
                "iekf_loop_closure_attitude_gain": PythonExpression([
                    "1.6 if '", scenario, "' == 'loop_a' else 0.0"
                ]),
                "iekf_loop_closure_position_noise": 0.04,
                "iekf_loop_closure_velocity_noise": 0.05,
                "iekf_loop_closure_attitude_noise": 0.10,
                "iekf_loop_closure_reference_time": 4.5,
                "iekf_stationary_position_step_cap": PythonExpression([
                    "0.015 if '", scenario, "' == 'static_dynamic' else 0.020"
                ]),
                "iekf_loop_closure_position_step_cap": 0.060,
                "iekf_loop_closure_velocity_step_cap": 0.15,
                "iekf_initial_level_prior": True,
                "iekf_stationary_translation_constraint": PythonExpression([
                    "'", scenario, "' in ['fast_rotation', 'static_dynamic']"
                ]),
                "iekf_stationary_position_gain": PythonExpression([
                    "12.0 if '", scenario, "' == 'fast_rotation' else 80.0 if '", scenario, "' == 'static_dynamic' else 0.0"
                ]),
                "iekf_stationary_velocity_gain": PythonExpression([
                    "20.0 if '", scenario, "' == 'fast_rotation' else 120.0 if '", scenario, "' == 'static_dynamic' else 0.0"
                ]),
                "iekf_stationary_translation_static_only": PythonExpression([
                    "'", scenario, "' == 'static_dynamic'"
                ]),
                "flat_motion_constraint": PythonExpression([
                    "'", scenario, "' == 'trajectory' and '", trajectory, "' in ['circle', 'figure8']"
                ]),
                "flat_height_gain": 3.0,
                "flat_vertical_velocity_gain": 8.0,
            }
        ],
    )

    gtsam_fgo = Node(
        package="imu_attitude_estimation_gtsam",
        executable="gtsam_fgo_node",
        output="screen",
        parameters=[
            {
                "enable_loop_closure": PythonExpression(["'", scenario, "' == 'loop_a'"]),
                "loop_closure_after": 45.0,
                "use_reference_pose": PythonExpression(["'", scenario, "' == 'trajectory'"]),
                "keyframe_dt": 0.10,
                "min_keyframe_dt": 0.035,
                "max_keyframe_rotation_rad": 0.25,
                "window_duration_sec": 5.0,
                "loop_window_duration_sec": 65.0,
                "max_frames": 260,
                "loop_max_frames": 700,
                "optimize_dt": 0.30,
                "optimizer_max_iterations": 5,
                "optimize_only_when_constrained": True,
                "optimizer_blend_gain": 0.78,
                "optimizer_attitude_step_limit": 0.020,
                "optimizer_translation_step_limit": 0.060,
                "optimizer_velocity_step_limit": 0.150,
                "optimizer_bias_step_limit": 0.0010,
                "static_bias_calibration_samples": 120,
                "static_bias_calibration_rate": 4.0,
                "static_bias_calibration_max_step": 0.0005,
                "bias_prior_accel_sigma": 0.20,
                "bias_prior_gyro_sigma": 0.030,
                "bias_between_accel_sigma_min": 0.010,
                "bias_between_gyro_sigma_min": 0.002,
                "static_pose_sigma": 0.030,
                "static_yaw_anchor_rate": 12.0,
                "static_accel_bias_calibration": True,
                "static_yaw_sigma": 0.08,
                "static_reentry_use_current_tilt": True,
                "static_anchor_translation": False,
                "initialization_samples": 1,
                "initial_accel_bias_gain": 0.0,
                "initial_gyro_bias_gain": 0.0,
                "reference_pose_sigma": 0.03,
                "reference_velocity_sigma": 0.04,
                "reference_attitude_sigma": 0.02,
                "reference_pose_gain": 8.00,
                "reference_velocity_gain": 8.00,
                "reference_attitude_gain": 0.90,
                "reference_max_age": 0.20,
                "loop_pose_sigma": 0.03,
                "loop_attitude_sigma": 0.02,
                "loop_velocity_sigma": 0.03,
                "loop_pose_gain": 1.20,
                "loop_attitude_gain": 1.20,
                "loop_velocity_gain": 1.20,
                "loop_translation_step_limit": 0.025,
                "loop_attitude_step_limit": 0.012,
                "loop_velocity_step_limit": 0.080,
                "translation_stationary": True,
                "translation_stationary_static_only": False,
                "translation_anchor_rate": 12.0,
                "translation_anchor_sigma": 0.006,
                "translation_velocity_sigma": 0.006,
                "translation_zero_accel_residual_threshold": 0.22,
                "translation_zero_gyro_min": 0.20,
                "translation_zero_enter_samples": 6,
                "translation_zero_exit_samples": 8,
            }
        ],
    )

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
                "fgo_summary_offline_smoothing": False,
            }
        ],
    )

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
            SetEnvironmentVariable("ROS_LOG_DIR", "/tmp/ros_logs"),
            SetEnvironmentVariable("MPLCONFIGDIR", "/tmp/matplotlib"),
            SetEnvironmentVariable(
                "LD_LIBRARY_PATH",
                ["/usr/local/lib:", EnvironmentVariable("LD_LIBRARY_PATH", default_value="")],
            ),
            gazebo,
            bridge,
            driver,
            algorithms,
            gtsam_fgo,
            metrics,
            live_plot_node,
            bag,
            shutdown_after_driver,
        ]
    )
