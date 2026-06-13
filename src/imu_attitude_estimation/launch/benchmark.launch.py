import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
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
            }
        ],
    )

    live_plot_node = Node(
        package="imu_attitude_estimation",
        executable="live_plot_node",
        output="screen",
        condition=IfCondition(live_plot),
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
        ]
    )
