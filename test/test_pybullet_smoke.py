from dvrk_pybullet.robot import load_robot, reset_joint_positions
from dvrk_pybullet.urdf_materializer import materialize_virtual_psm
import pytest


def test_virtual_psm1_loads_in_direct_mode(tmp_path):
    pybullet = pytest.importorskip("pybullet")
    connection = pybullet.connect(pybullet.DIRECT)
    assert connection >= 0
    try:
        artifact = materialize_virtual_psm(generated_root=tmp_path)
        names = ("yaw", "pitch", "insertion", "roll", "wrist_pitch", "wrist_yaw")
        robot = load_robot(pybullet, artifact.urdf_path, names)
        reset_joint_positions(pybullet, robot, (0.0, 0.0, 0.12, 0.0, 0.0, 0.0))
        measured = tuple(
            pybullet.getJointState(robot.body_id, index)[0]
            for index in robot.controlled_joint_indices
        )
        assert measured == pytest.approx((0.0, 0.0, 0.12, 0.0, 0.0, 0.0))
    finally:
        pybullet.disconnect(connection)
