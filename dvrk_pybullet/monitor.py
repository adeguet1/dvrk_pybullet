"""Optional PyQt6 monitor for a running PyBullet simulator world."""

from __future__ import annotations

import time


class PyBulletMonitor:
    """Keep UI work out of simulation/ROS logic while exposing basic recovery."""

    def __init__(self, runtime) -> None:
        from PyQt6 import QtWidgets

        self.application = (
            QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        )
        self.runtime = runtime
        self.window = QtWidgets.QWidget()
        self.window.setWindowTitle("dVRK PyBullet Monitor")
        layout = QtWidgets.QVBoxLayout(self.window)
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
        self.arm_labels = {}
        for name in runtime.arms:
            arm_box = QtWidgets.QGroupBox(name)
            arm_layout = QtWidgets.QVBoxLayout(arm_box)
            state_commands = QtWidgets.QComboBox()
            state_commands.addItem("State command…", None)
            for command in (
                "enable", "disable", "pause", "resume", "home", "unhome",
                "fault", "clear_fault",
            ):
                state_commands.addItem(command, command)
            state_commands.activated.connect(
                lambda index, arm=name, combo=state_commands: self._state_command(
                    arm, combo, index
                )
            )
            arm_layout.addWidget(state_commands)
            label = QtWidgets.QLabel()
            label.setWordWrap(True)
            arm_layout.addWidget(label)
            layout.addWidget(arm_box)
            self.arm_labels[name] = label
        self.window.show()
        self._last_update = 0.0

    def _state_command(self, arm_name: str, combo, index: int) -> None:
        command = combo.itemData(index)
        combo.setCurrentIndex(0)
        if command is not None:
            self.runtime.arms[arm_name].commands.submit_discrete(
                "state_command", command
            )

    def update(self, snapshots, step_hz: float) -> None:
        self.application.processEvents()
        now = time.monotonic()
        if now - self._last_update < 0.2:
            return
        self._last_update = now
        grasp_count = (
            len(self.runtime.grasp_manager.attachments)
            if self.runtime.grasp_manager is not None
            else 0
        )
        self.performance.setText(
            f"Simulation: {step_hz:.1f} Hz; grasps: {grasp_count}"
        )
        for name, snapshot in snapshots.items():
            joints = ", ".join(
                f"{value:.3f}" for value in snapshot.measured_js.position
            )
            self.arm_labels[name].setText(
                f"{name}: {snapshot.operating_state.state}; "
                f"homed={snapshot.operating_state.is_homed}\n{joints}"
            )
