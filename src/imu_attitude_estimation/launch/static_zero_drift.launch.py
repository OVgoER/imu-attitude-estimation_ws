from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("imu_attitude_estimation"), "launch", "benchmark.launch.py"]
                    )
                ),
                launch_arguments={
                    "scenario": "static_zero_drift",
                    "duration": "60.0",
                    "sync_gazebo_pose": "false",
                }.items(),
            )
        ]
    )
