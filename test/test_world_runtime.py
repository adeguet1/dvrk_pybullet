from pathlib import Path
from types import SimpleNamespace

import pytest

from dvrk_simulator_base.command_mailbox import CommandMailboxes
from dvrk_simulator_base.config import load_robot_config

from dvrk_pybullet.runtime import RuntimeOptions
from dvrk_pybullet.camera import CameraOptions
from dvrk_pybullet.world_runtime import PyBulletWorldRuntime


class _FrameCollector:
    def __init__(self, options):
        self.frames = []

    def start(self):
        pass

    def push(self, frame):
        self.frames.append(frame)

    def close(self):
        pass


def test_egl_repeated_mesh_color_survives_missing_cached_metadata(tmp_path):
    """EGL omits mesh metadata and uses its palette on the second instance."""
    gray = (0.6, 0.6, 0.6, 1.0)
    yellow = (0.95, 0.75, 0.05, 1.0)
    red = (1.0, 0.0, 0.0, 1.0)
    mesh_path = str(tmp_path / "instrument.obj")
    calls = []
    world = PyBulletWorldRuntime.__new__(PyBulletWorldRuntime)
    world._egl_plugin = 0
    world.connection = 7
    world.arms = {}
    for body in range(3):
        material = '<material name="red"><color rgba="1 0 0 1"/></material>' if body == 2 else ''
        urdf = tmp_path / f"robot-{body}.urdf"
        urdf.write_text(
            f'<robot name="test"><link name="jaw"><visual>'
            f'<geometry><mesh filename="{mesh_path}"/></geometry>'
            f'{material}</visual></link></robot>'
        )
        world.arms[str(body)] = SimpleNamespace(
            artifact=SimpleNamespace(urdf_path=urdf),
            robot=SimpleNamespace(body_id=body, link_indices={"jaw": 4}),
        )

    def shapes(body, physicsClientId):
        assert physicsClientId == 7
        # Only the first load reports real geometry/filename/dimensions.
        return [(body, 4, 5 if body == 0 else 0,
                 (1, 1, 1) if body == 0 else (0, 0, 0),
                 mesh_path.encode() if body == 0 else b'',
                 (0, 0, 0), (0, 0, 0, 1), (gray, yellow, red)[body])]

    def change(body, link, **kwargs):
        calls.append((body, link, kwargs))

    world.pybullet = SimpleNamespace(
        getVisualShapeData=shapes, changeVisualShape=change
    )
    world._restore_egl_mesh_material_colors()
    assert calls == [(1, 4, {"rgbaColor": gray, "physicsClientId": 7})]


def test_world_owns_one_connection_and_multiple_kinematic_arms(tmp_path):
    pytest.importorskip("pybullet")
    arm_root = Path(__file__).parents[2] / "dvrk-cp-base" / "share" / "arms"
    if not arm_root.is_dir():
        arm_root = Path("/tmp/dvrk-cp-base/share/arms")
    configs = (
        load_robot_config(
            arm_root / "PSM1.yaml", instrument="420006", base_position=[-0.1, 0, 0.17]
        ),
        load_robot_config(
            arm_root / "PSM2.yaml", instrument="420006", base_position=[0.1, 0, 0.17]
        ),
        load_robot_config(
            arm_root / "ECM.yaml", endoscope="Si_straight", base_position=[0, 0, 0.2]
        ),
    )
    commands = {config.name: CommandMailboxes() for config in configs}
    world = PyBulletWorldRuntime(
        configs,
        RuntimeOptions(generated_root=tmp_path),
        commands,
        camera_options=CameraOptions(
            renderer="tiny", width=16, height=12, rate_hz=30.0,
            socket_path=tmp_path / "camera.sock",
        ),
        video_sink_factory=_FrameCollector,
    )
    try:
        initial = world.initialize()
        assert set(initial) == {"PSM1", "PSM2", "ECM"}
        assert len({arm.robot.body_id for arm in world.arms.values()}) == 3
        assert all(arm.connection == world.connection for arm in world.arms.values())
        assert initial["ECM"].jaw_measured is None
        stepped = world.step()
        assert all(snapshot.sequence == 1 for snapshot in stepped.values())
        assert len(world.video_sink.frames) == 1
        assert world.video_sink.frames[0].rgba.shape == (12, 16, 4)
    finally:
        world.shutdown()
