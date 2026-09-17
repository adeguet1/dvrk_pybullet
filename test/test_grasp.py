from types import SimpleNamespace

from dvrk_pybullet.grasp import GraspManager


class FakePyBullet:
    JOINT_FIXED = 4

    def __init__(self):
        self.enabled = []
        self.removed = []

    def getContactPoints(self, **_kwargs):
        return (("contact",),)

    def getLinkState(self, *_args, **_kwargs):
        return (None, None, None, None, (0, 0, 0), (0, 0, 0, 1))

    def getBasePositionAndOrientation(self, *_args, **_kwargs):
        return (0, 0, 0), (0, 0, 0, 1)

    def invertTransform(self, *_args):
        return (0, 0, 0), (0, 0, 0, 1)

    def multiplyTransforms(self, *_args):
        return (0, 0, 0), (0, 0, 0, 1)

    def createConstraint(self, **_kwargs):
        return 7

    def changeConstraint(self, *_args, **_kwargs):
        pass

    def setCollisionFilterPair(self, *_args, **_kwargs):
        self.enabled.append(_args[4])

    def removeConstraint(self, constraint, **_kwargs):
        self.removed.append(constraint)


def test_two_jaw_contact_attaches_and_opening_releases_dynamic_object():
    bullet = FakePyBullet()
    arm = SimpleNamespace(
        config=SimpleNamespace(name="PSM1"),
        robot=SimpleNamespace(
            body_id=1,
            link_indices={"PSM1_jaw_1_link": 3, "PSM1_jaw_2_link": 4},
        ),
        tool_link_index=5,
    )
    objects = {"cube": SimpleNamespace(body_id=2, spec=SimpleNamespace(fixed=False))}
    manager = GraspManager(bullet, 0, {"PSM1": arm}, objects)
    manager.step({"PSM1": SimpleNamespace(jaw_measured=0.0)})
    assert manager.attachments["PSM1"].object_name == "cube"
    assert bullet.enabled == [0, 0]
    manager.step({"PSM1": SimpleNamespace(jaw_measured=0.2)})
    assert manager.attachments == {}
    assert bullet.removed == [7]
    assert bullet.enabled == [0, 0, 1, 1]
