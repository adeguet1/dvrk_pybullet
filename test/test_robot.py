from pathlib import Path

from dvrk_pybullet.errors import PyBulletBackendError
from dvrk_pybullet.robot import (
    load_robot,
    reset_joint_positions,
    reset_joint_with_mimics,
)
import pytest


class FakePyBullet:

    def __init__(self, names=("yaw", "pitch", "insertion")):
        self.reset_calls = []
        self.names = names


    def loadURDF(self, path, **kwargs):
        assert Path(path).is_file()
        assert kwargs["useFixedBase"] is True
        return 7

    def getBodyInfo(self, body_id):
        assert body_id == 7
        return (b"world", b"PSM1")

    def getNumJoints(self, body_id):
        return len(self.names)

    def getJointInfo(self, body_id, index):
        values = [None] * 13
        values[1] = self.names[index].encode()
        values[12] = f"{self.names[index]}_link".encode()
        return tuple(values)

    def resetJointState(self, body_id, index, position, targetVelocity=0.0):
        self.reset_calls.append((body_id, index, position, targetVelocity))


def test_robot_mapping_and_reset_follow_configured_name_order(tmp_path):
    urdf = tmp_path / "model.urdf"
    urdf.write_text("<robot name='test'/>", encoding="utf-8")
    client = FakePyBullet()
    robot = load_robot(client, urdf, ("insertion", "yaw"))

    assert robot.controlled_joint_indices == (2, 0)
    assert robot.link_indices["world"] == -1
    reset_joint_positions(client, robot, (0.12, 0.1))
    assert client.reset_calls == [(7, 2, 0.12, 0.0), (7, 0, 0.1, 0.0)]


def test_missing_configured_joint_is_rejected(tmp_path):
    urdf = tmp_path / "model.urdf"
    urdf.write_text("<robot name='test'/>", encoding="utf-8")
    with pytest.raises(PyBulletBackendError, match="wrist_yaw"):
        load_robot(FakePyBullet(), urdf, ("yaw", "wrist_yaw"))


def test_urdf_mimic_formula_is_applied_by_name(tmp_path):
    urdf = tmp_path / "model.urdf"
    urdf.write_text(
        """<robot name="test">
  <joint name="jaw" type="revolute"/>
  <joint name="jaw_1" type="revolute"><mimic joint="jaw" multiplier="0.5"/></joint>
  <joint name="jaw_2" type="revolute"><mimic joint="jaw" multiplier="-0.5" offset="0.1"/></joint>
</robot>""",
        encoding="utf-8",
    )
    client = FakePyBullet(("jaw", "jaw_1", "jaw_2"))
    robot = load_robot(client, urdf, ("jaw",))
    reset_joint_with_mimics(client, robot, "jaw", 0.4, velocity=0.2)

    assert client.reset_calls == [
        (7, 0, 0.4, 0.2),
        (7, 1, 0.2, 0.1),
        (7, 2, -0.1, -0.1),
    ]


def test_controlled_joint_reset_also_updates_its_mimics(tmp_path):
    urdf = tmp_path / "model.urdf"
    urdf.write_text(
        """<robot name="test">
  <joint name="pitch" type="revolute"/>
  <joint name="pitch_linkage" type="revolute">
    <mimic joint="pitch" multiplier="-1" offset="0.2"/>
  </joint>
</robot>""",
        encoding="utf-8",
    )
    client = FakePyBullet(("pitch", "pitch_linkage"))
    robot = load_robot(client, urdf, ("pitch",))
    reset_joint_positions(client, robot, (0.5,), (0.1,))

    assert client.reset_calls == [
        (7, 0, 0.5, 0.1),
        (7, 1, -0.3, -0.1),
    ]
