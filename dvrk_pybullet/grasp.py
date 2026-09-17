"""Contact-qualified, constraint-backed grasping for kinematic PSMs."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


@dataclass(frozen=True)
class GraspAttachment:
    arm_name: str
    object_name: str
    constraint_id: int
    jaw_link_indices: tuple[int, int]


class GraspManager:
    """Attach a dynamic object only when both PSM jaws contact while closed."""

    def __init__(
        self,
        pybullet,
        connection: int,
        arms: Mapping,
        scene_objects: Mapping,
        *,
        close_threshold: float = 0.05,
        release_threshold: float = 0.15,
        max_force: float = 20.0,
    ) -> None:
        self.pybullet = pybullet
        self.connection = connection
        self.arms = arms
        self.objects = {
            name: item for name, item in scene_objects.items() if not item.spec.fixed
        }
        self.close_threshold = close_threshold
        self.release_threshold = release_threshold
        self.max_force = max_force
        self.attachments: dict[str, GraspAttachment] = {}

    def step(self, snapshots) -> None:
        """Release opened grippers, then seek one two-jaw contact per PSM."""
        for arm_name, attachment in tuple(self.attachments.items()):
            jaw = snapshots[arm_name].jaw_measured
            if jaw is None or jaw >= self.release_threshold:
                self._release(attachment)
        held_objects = {item.object_name for item in self.attachments.values()}
        for arm_name, arm in self.arms.items():
            if arm_name in self.attachments or arm_name not in snapshots:
                continue
            snapshot = snapshots[arm_name]
            if snapshot.jaw_measured is None or snapshot.jaw_measured > self.close_threshold:
                continue
            jaw_links = self._jaw_links(arm)
            if jaw_links is None:
                continue
            for object_name, item in self.objects.items():
                if object_name in held_objects:
                    continue
                if self._both_jaws_contact(arm, item.body_id, jaw_links):
                    self._attach(arm_name, arm, object_name, item.body_id, jaw_links)
                    held_objects.add(object_name)
                    break

    def release_all(self) -> None:
        for attachment in tuple(self.attachments.values()):
            self._release(attachment)

    @staticmethod
    def _jaw_links(arm) -> tuple[int, int] | None:
        prefix = f"{arm.config.name}_"
        links = arm.robot.link_indices
        names = (f"{prefix}jaw_1_link", f"{prefix}jaw_2_link")
        if any(name not in links for name in names):
            return None
        return links[names[0]], links[names[1]]

    def _both_jaws_contact(self, arm, object_id: int, jaw_links: tuple[int, int]) -> bool:
        return all(
            bool(
                self.pybullet.getContactPoints(
                    bodyA=arm.robot.body_id,
                    bodyB=object_id,
                    linkIndexA=link_index,
                    physicsClientId=self.connection,
                )
            )
            for link_index in jaw_links
        )

    def _attach(self, arm_name, arm, object_name, object_id, jaw_links) -> None:
        tool_state = self.pybullet.getLinkState(
            arm.robot.body_id,
            arm.tool_link_index,
            computeForwardKinematics=True,
            physicsClientId=self.connection,
        )
        object_position, object_orientation = self.pybullet.getBasePositionAndOrientation(
            object_id, physicsClientId=self.connection
        )
        inverse = self.pybullet.invertTransform(tool_state[4], tool_state[5])
        relative = self.pybullet.multiplyTransforms(
            inverse[0], inverse[1], object_position, object_orientation
        )
        constraint_id = self.pybullet.createConstraint(
            parentBodyUniqueId=arm.robot.body_id,
            parentLinkIndex=arm.tool_link_index,
            childBodyUniqueId=object_id,
            childLinkIndex=-1,
            jointType=self.pybullet.JOINT_FIXED,
            jointAxis=(0.0, 0.0, 0.0),
            parentFramePosition=relative[0],
            parentFrameOrientation=relative[1],
            childFramePosition=(0.0, 0.0, 0.0),
            childFrameOrientation=(0.0, 0.0, 0.0, 1.0),
            physicsClientId=self.connection,
        )
        self.pybullet.changeConstraint(
            constraint_id, maxForce=self.max_force, physicsClientId=self.connection
        )
        for link_index in jaw_links:
            self.pybullet.setCollisionFilterPair(
                arm.robot.body_id,
                object_id,
                link_index,
                -1,
                0,
                physicsClientId=self.connection,
            )
        self.attachments[arm_name] = GraspAttachment(
            arm_name, object_name, constraint_id, jaw_links
        )

    def _release(self, attachment: GraspAttachment) -> None:
        arm = self.arms[attachment.arm_name]
        item = self.objects[attachment.object_name]
        self.pybullet.removeConstraint(attachment.constraint_id, physicsClientId=self.connection)
        for link_index in attachment.jaw_link_indices:
            self.pybullet.setCollisionFilterPair(
                arm.robot.body_id,
                item.body_id,
                link_index,
                -1,
                1,
                physicsClientId=self.connection,
            )
        del self.attachments[attachment.arm_name]
