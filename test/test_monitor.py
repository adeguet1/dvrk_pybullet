import math
import os
from types import SimpleNamespace
import numpy as np
import pytest

os.environ["QT_QPA_PLATFORM"] = "offscreen"

try:
    from PyQt6 import QtWidgets
    from dvrk_pybullet.monitor import PyBulletMonitor, PyBulletSimulatorWidget
except (ImportError, SystemExit):
    pytest.skip("PyQt6 is required for monitor tests", allow_module_level=True)

from dvrk_simulator_base.config import JointConfig, RobotConfig
from dvrk_simulator_base.snapshots import ArmSnapshot, OperatingStateSnapshot
from dvrk_simulator_base.types import JointState, Pose, Twist


class DummyMailbox:
    def __init__(self):
        self.submissions = []

    def submit_discrete(self, name, command):
        self.submissions.append((name, command))


def make_arm(name, arm_type="PSM"):
    joints = (
        JointConfig("yaw", "revolute", -1.5, 1.5, 1.0),
        JointConfig("insertion", "prismatic", 0.0, 0.24, 0.4),
    )
    config = RobotConfig(
        name=name,
        type=arm_type,
        model=f"Virtual/{name}.urdf.xacro",
        instrument="420006" if arm_type == "PSM" else None,
        endoscope="Si_straight" if arm_type == "ECM" else None,
        parent_frame="world",
        base_frame=f"{name}_base",
        rcm_frame=f"{name}_RCM",
        tool_frame=f"{name}_tool",
        adaptor_frame=f"{name}_adaptor",
        base_position=np.zeros(3),
        base_orientation_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        joints=joints,
        home_position=np.array([0.0, 0.1]),
        raw={},
    )
    return SimpleNamespace(config=config, commands=DummyMailbox())


@pytest.fixture
def dummy_runtime():
    runtime = SimpleNamespace(
        arms={"ECM": make_arm("ECM", "ECM"), "PSM1": make_arm("PSM1", "PSM")},
        request_reset_called=False,
        collision_debug_value=None,
        grasp_manager=SimpleNamespace(attachments={1: None, 2: None}),
    )

    def request_reset():
        runtime.request_reset_called = True

    def set_collision_debug(val):
        runtime.collision_debug_value = val

    runtime.request_reset = request_reset
    runtime.set_collision_debug = set_collision_debug
    return runtime


def test_pybullet_monitor_tabs_and_controls(dummy_runtime):
    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monitor = PyBulletMonitor(dummy_runtime)

    assert monitor.tabs.count() == 3
    assert monitor.tabs.tabText(0) == "Simulator"
    assert monitor.tabs.tabText(1) == "ECM"
    assert monitor.tabs.tabText(2) == "PSM1"

    # Verify reset button on simulator tab
    sim_tab = monitor.simulator_widget
    reset_btn = sim_tab.findChild(QtWidgets.QPushButton)
    assert reset_btn is not None
    reset_btn.click()
    assert dummy_runtime.request_reset_called

    # Verify collision checkbox on simulator tab
    collision_cb = sim_tab.findChild(QtWidgets.QCheckBox)
    assert collision_cb is not None
    collision_cb.setChecked(True)
    assert dummy_runtime.collision_debug_value is True

    # Test state command dispatch from arm tab
    psm_widget = monitor.arm_widgets["PSM1"]
    home_index = psm_widget.state_combo.findData("home")
    assert home_index > 0
    psm_widget.state_combo.setCurrentIndex(home_index)
    psm_widget.state_combo.activated.emit(home_index)

    assert ("state_command", "home") in dummy_runtime.arms["PSM1"].commands.submissions

    # Test joint command dispatch from arm tab
    psm_widget._joint_spinners[0].setValue(15.0)   # yaw 15 deg
    psm_widget._joint_spinners[1].setValue(60.0)   # insertion 60 mm
    psm_widget._joint_spinners[2].setValue(10.0)   # jaw 10 deg
    psm_widget.joint_apply_btn.click()

    submissions = dummy_runtime.arms["PSM1"].commands.submissions
    move_jp = next(cmd for name, cmd in submissions if name == "move_jp")
    assert np.isclose(move_jp[0], math.radians(15.0))
    assert np.isclose(move_jp[1], 0.06)

    jaw_move_jp = next(cmd for name, cmd in submissions if name == "jaw/move_jp")
    assert np.isclose(jaw_move_jp, math.radians(10.0))

    # Test Cartesian command dispatch from arm tab
    psm_widget._cart_spinners[0].setValue(120.0)  # X: 120 mm
    psm_widget.cart_apply_btn.click()

    move_cp = next(cmd for name, cmd in submissions if name == "move_cp")
    assert np.isclose(move_cp.position[0], 0.12)


def test_pybullet_monitor_update(dummy_runtime):
    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monitor = PyBulletMonitor(dummy_runtime)

    snapshot_ecm = ArmSnapshot(
        sequence=1,
        simulation_time=0.1,
        valid=True,
        measured_js=JointState(("yaw", "insertion"), np.array([0.1, 0.02]), np.zeros(2)),
        setpoint_js=JointState(("yaw", "insertion"), np.zeros(2), np.zeros(2)),
        measured_cp_world=Pose(np.array([0.01, 0.02, 0.03]), np.eye(3)),
        setpoint_cp_world=Pose(np.zeros(3), np.eye(3)),
        measured_cv_world=Twist(np.zeros(3), np.zeros(3)),
        jaw_measured=None,
        jaw_setpoint=None,
        operating_state=OperatingStateSnapshot("ENABLED", True, False),
    )

    snapshot_psm1 = ArmSnapshot(
        sequence=1,
        simulation_time=0.1,
        valid=True,
        measured_js=JointState(("yaw", "insertion"), np.array([0.2, 0.05]), np.zeros(2)),
        setpoint_js=JointState(("yaw", "insertion"), np.zeros(2), np.zeros(2)),
        measured_cp_world=Pose(np.array([0.1, -0.2, 0.3]), np.eye(3)),
        setpoint_cp_world=Pose(np.zeros(3), np.eye(3)),
        measured_cv_world=Twist(np.zeros(3), np.zeros(3)),
        jaw_measured=0.3,
        jaw_setpoint=0.3,
        operating_state=OperatingStateSnapshot("ENABLED", True, False),
    )

    snapshots = {"ECM": snapshot_ecm, "PSM1": snapshot_psm1}

    # When Simulator tab is active:
    monitor.tabs.setCurrentIndex(0)
    monitor.update(snapshots, 120.0)

    assert "Simulation: 120.0 Hz; grasps: 2" in monitor.simulator_widget.performance.text()

    # When switching to PSM1 tab:
    monitor.tabs.setCurrentIndex(2)
    # The tab change handler should update PSM1 immediately:
    psm_widget = monitor.arm_widgets["PSM1"]
    assert psm_widget.state_label.text() == "State: ENABLED"
    assert psm_widget.joint_table.item(0, 1).text() == "50.00"  # 0.05 m -> 50.00 mm
    assert psm_widget.cart_table.item(0, 0).text() == "100.00"  # 0.1 m -> 100.00 mm
