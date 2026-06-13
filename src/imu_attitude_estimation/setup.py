from glob import glob
import os

from setuptools import find_packages, setup

package_name = "imu_attitude_estimation"


def data_files_for(directory):
    return [
        (
            os.path.join("share", package_name, directory),
            glob(os.path.join(directory, "*")),
        )
    ]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
        *data_files_for("launch"),
        *data_files_for("config"),
        *data_files_for("worlds"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zjy",
    maintainer_email="zjy@example.com",
    description="ROS2 and Gazebo 6-axis IMU attitude and inertial-state estimation benchmarks.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "algorithm_runner = imu_attitude_estimation.algorithm_runner:main",
            "benchmark_driver = imu_attitude_estimation.benchmark_driver:main",
            "live_plot_node = imu_attitude_estimation.live_plot_node:main",
            "metrics_node = imu_attitude_estimation.metrics_node:main",
            "report_generator = imu_attitude_estimation.report_generator:main",
            "run_all_benchmarks = imu_attitude_estimation.run_all_benchmarks:main",
            "synthetic_benchmark = imu_attitude_estimation.synthetic_benchmark:main",
        ],
    },
)
