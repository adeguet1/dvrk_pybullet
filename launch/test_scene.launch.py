"""Run a bounded headless PyBullet scene smoke test."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("dvrk_pybullet"))
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(share / "share" / "pybullet.yaml")),
        DeclareLaunchArgument("scene", description="Scene YAML path or installed scene filename"),
        DeclareLaunchArgument("timeout", default_value="1.0", description="Maximum test run duration in seconds"),
        SetEnvironmentVariable("DVRK_SIMULATOR_TEST_TIMEOUT", LaunchConfiguration("timeout")),
        Node(
            package="dvrk_pybullet", executable="simulator_node", output="screen",
            arguments=["--config", LaunchConfiguration("config"), "--scene", LaunchConfiguration("scene")],
        ),
    ])