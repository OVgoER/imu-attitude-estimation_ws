from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    trajectory = LaunchConfiguration("trajectory")
    return LaunchDescription(
        [
            DeclareLaunchArgument("trajectory", default_value="circle"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("imu_attitude_estimation"), "launch", "benchmark.launch.py"]
                    )
                ),
                launch_arguments={
                    "scenario": "trajectory",
                    "trajectory": trajectory,
                    "duration": "60.0",
                }.items(),
            ),
        ]
    )
