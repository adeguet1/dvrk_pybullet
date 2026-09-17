"""Optional PyQt6 monitor for a running PyBullet simulator world."""

from __future__ import annotations

import time
from typing import Any

import numpy as np
from PyQt6 import QtWidgets

from dvrk_simulator_base.arm_widget import SimulatorArmWidget
from dvrk_simulator_base.types import Pose


class PyBulletSimulatorWidget(QtWidgets.QWidget):
    """Simulator-specific general information and recovery controls."""

    def __init__(self, runtime: Any, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.runtime = runtime

        layout = QtWidgets.QVBoxLayout(self)

        buttons = QtWidgets.QHBoxLayout()
        reset = QtWidgets.QPushButton("Reset scene")
        reset.clicked.connect(runtime.request_reset)
        self.collision = QtWidgets.QCheckBox("Show collision shapes")
        self.collision.toggled.connect(runtime.set_collision_debug)
        buttons.addWidget(reset)
        buttons.addWidget(self.collision)
        buttons.addStretch()
        layout.addLayout(buttons)

        self.performance = QtWidgets.QLabel("Simulation: collecting…")
        layout.addWidget(self.performance)
        layout.addStretch()

    def update_status(self, step_hz: float, grasp_count: int) -> None:
        if not self.isVisible():
            return
        self.performance.setText(
            f"Simulation: {step_hz:.1f} Hz; grasps: {grasp_count}"
        )


class PyBulletMonitor:
    """Keep UI work out of simulation/ROS logic while exposing basic recovery."""

    def __init__(self, runtime: Any) -> None:
        self.application = (
            QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        )
        self.runtime = runtime
        self.window = QtWidgets.QWidget()
        self.window.setWindowTitle("dVRK PyBullet Monitor")
        self.window.resize(750, 360)
        layout = QtWidgets.QVBoxLayout(self.window)

        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs)

        # Tab 0: Simulator general info
        self.simulator_widget = PyBulletSimulatorWidget(runtime)
        self.tabs.addTab(self.simulator_widget, "Simulator")

        # Tabs 1..N: One tab per arm
        self.arm_widgets: dict[str, SimulatorArmWidget] = {}
        for name, arm in runtime.arms.items():
            arm_widget = SimulatorArmWidget(
                arm.config,
                on_state_command=lambda command, arm_name=name: self._state_command(
                    arm_name, command
                ),
                on_joint_command=lambda positions, jaw=None, arm_name=name: self._joint_command(
                    arm_name, positions, jaw
                ),
                on_cartesian_command=lambda pose, arm_name=name: self._cartesian_command(
                    arm_name, pose
                ),
            )
            self.tabs.addTab(arm_widget, name)
            self.arm_widgets[name] = arm_widget

        self._latest_snapshots: dict[str, Any] = {}
        self._latest_step_hz = 0.0
        self._latest_grasp_count = 0
        self.tabs.currentChanged.connect(self._on_tab_changed)

        self.window.show()
        self._last_update = 0.0

    def _on_tab_changed(self, index: int) -> None:
        current_widget = self.tabs.widget(index)
        if current_widget is self.simulator_widget:
            self.simulator_widget.update_status(
                self._latest_step_hz, self._latest_grasp_count
            )
        else:
            for name, widget in self.arm_widgets.items():
                if widget is current_widget and name in self._latest_snapshots:
                    widget.update_snapshot(self._latest_snapshots[name])

    def _state_command(self, arm_name: str, command: str) -> None:
        if command is not None:
            self.runtime.arms[arm_name].commands.submit_discrete(
                "state_command", command
            )

    def _joint_command(
        self, arm_name: str, positions: list[float], jaw: float | None = None
    ) -> None:
        self.runtime.arms[arm_name].commands.submit_discrete(
            "move_jp", np.asarray(positions, dtype=float)
        )
        if jaw is not None and getattr(self.runtime.arms[arm_name].config, "type", "") == "PSM":
            self.runtime.arms[arm_name].commands.submit_discrete(
                "jaw/move_jp", float(jaw)
            )

    def _cartesian_command(self, arm_name: str, pose: Pose) -> None:
        self.runtime.arms[arm_name].commands.submit_discrete(
            "move_cp", pose
        )

    def update(self, snapshots: dict[str, Any], step_hz: float) -> None:
        self.application.processEvents()
        self._latest_snapshots = snapshots
        self._latest_step_hz = step_hz
        self._latest_grasp_count = (
            len(self.runtime.grasp_manager.attachments)
            if self.runtime.grasp_manager is not None
            else 0
        )

        now = time.monotonic()
        if now - self._last_update < 0.2:
            return
        self._last_update = now

        self.simulator_widget.update_status(step_hz, self._latest_grasp_count)
        for name, snapshot in snapshots.items():
            widget = self.arm_widgets.get(name)
            if widget is not None:
                widget.update_snapshot(snapshot)
