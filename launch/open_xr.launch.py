"""Start the PyBullet patient cart and the optional Quest/OpenXR console."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("dvrk_pybullet"))
    open_xr_directory = package_share / "share" / "open-xr"
    simulator_config = open_xr_directory / "pybullet.yaml"
    scene = package_share / "share" / "scenes" / "ECM_PSM1_PSM2_PSM3.yaml"
    system_config = (
        open_xr_directory / "system-MTML-MTMR-OpenXR-patient-cart-ROS.json"
    )
    overlay_config = open_xr_directory / "dvrk-console-overlay.json"
    simulator = Node(
        package="dvrk_pybullet",
        executable="simulator",
        output="screen",
        arguments=[
            "--config", str(simulator_config),
            "--scene", str(scene),
            "--gui", LaunchConfiguration("gui"),
        ],
    )
    dvrk_system = Node(
        package="dvrk_robot",
        executable="dvrk_system",
        name="dvrk_system",
        output="screen",
        cwd=str(open_xr_directory),
        arguments=["--json-config", str(system_config)],
    )
    console_overlay = Node(
        package="dvrk_console",
        executable="stereo_display",
        name="pybullet_console_overlay",
        output="screen",
        arguments=["-c", str(overlay_config)],
    )

    stop_with_simulator = RegisterEventHandler(
        OnProcessExit(
            target_action=simulator,
            on_exit=[EmitEvent(event=Shutdown(reason="PyBullet simulator exited"))],
        )
    )
    stop_with_console = RegisterEventHandler(
        OnProcessExit(
            target_action=dvrk_system,
            on_exit=[EmitEvent(event=Shutdown(reason="dvrk_system exited"))],
        )
    )
    stop_with_overlay = RegisterEventHandler(
        OnProcessExit(
            target_action=console_overlay,
            on_exit=[EmitEvent(event=Shutdown(reason="console video overlay exited"))],
        )
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="show the PyBullet desktop GUI",
            ),
            simulator,
            console_overlay,
            dvrk_system,
            stop_with_simulator,
            stop_with_console,
            stop_with_overlay,
        ]
    )
