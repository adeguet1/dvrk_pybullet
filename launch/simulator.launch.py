"""Start one configured dVRK PyBullet scene."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("dvrk_pybullet"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "config", default_value=str(share / "share" / "pybullet.yaml"),
            description="Backend runtime configuration YAML",
        ),
        DeclareLaunchArgument(
            "scene", description="Scene YAML path or installed scene filename",
        ),
        Node(
            package="dvrk_pybullet", executable="simulator_node", output="screen",
            arguments=["--config", LaunchConfiguration("config"), "--scene", LaunchConfiguration("scene")],
        ),
    ])