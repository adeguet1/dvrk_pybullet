"""PyBullet GUI visualization of the collision geometry declared in URDF."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


class CollisionShapeOverlay:
    """Render translucent, non-colliding copies of every URDF collision shape."""

    _COLOR = (0.05, 0.85, 1.0, 0.20)

    def __init__(self, pybullet, connection: int) -> None:
        self.pybullet = pybullet
        self.connection = connection
        self._shapes = []

    def add_urdf(self, body_id: int, urdf_path: str | Path) -> None:
        """Register collision geometries from the exact URDF loaded by Bullet."""
        root = ET.parse(urdf_path).getroot()
        link_indices = self._link_indices(body_id)
        for link in root.findall("link"):
            link_index = link_indices.get(link.attrib.get("name", ""))
            if link_index is None:
                continue
            for collision in link.findall("collision"):
                shape = self._create_visual(collision.find("geometry"))
                if shape < 0:
                    continue
                position, orientation = self._origin(collision.find("origin"))
                visual_body = self.pybullet.createMultiBody(
                    baseMass=0.0, baseCollisionShapeIndex=-1,
                    baseVisualShapeIndex=shape, physicsClientId=self.connection,
                )
                self._shapes.append((visual_body, body_id, link_index, position, orientation))

    def clear(self) -> None:
        for item in self._shapes:
            self.pybullet.removeBody(item[0], physicsClientId=self.connection)
        self._shapes = []

    def update(self) -> None:
        """Move each visual copy with the corresponding owning URDF link."""
        for visual_body, body_id, link_index, position, orientation in self._shapes:
            link_position, link_orientation = self._link_pose(body_id, link_index)
            world_position, world_orientation = self.pybullet.multiplyTransforms(
                link_position, link_orientation, position, orientation
            )
            self.pybullet.resetBasePositionAndOrientation(
                visual_body, world_position, world_orientation,
                physicsClientId=self.connection,
            )

    def _link_indices(self, body_id: int) -> dict[str, int]:
        base = self.pybullet.getBodyInfo(body_id)
        result = {self._decode(base[0]): -1}
        for index in range(self.pybullet.getNumJoints(body_id)):
            result[self._decode(self.pybullet.getJointInfo(body_id, index)[12])] = index
        return result

    def _create_visual(self, geometry) -> int:
        if geometry is None or not list(geometry):
            return -1
        element = list(geometry)[0]
        common = {"rgbaColor": self._COLOR, "physicsClientId": self.connection}
        if element.tag == "box":
            size = self._numbers(element.attrib.get("size"), 3, (0.0, 0.0, 0.0))
            return self.pybullet.createVisualShape(
                self.pybullet.GEOM_BOX,
                halfExtents=[value * 0.5 for value in size], **common,
            )
        if element.tag == "sphere":
            return self.pybullet.createVisualShape(
                self.pybullet.GEOM_SPHERE,
                radius=float(element.attrib["radius"]), **common,
            )
        if element.tag == "cylinder":
            return self.pybullet.createVisualShape(
                self.pybullet.GEOM_CYLINDER,
                radius=float(element.attrib["radius"]),
                length=float(element.attrib["length"]), **common,
            )
        if element.tag == "mesh":
            filename = element.attrib.get("filename")
            if not filename:
                return -1
            scale = self._numbers(element.attrib.get("scale"), 3, (1.0, 1.0, 1.0))
            return self.pybullet.createVisualShape(
                self.pybullet.GEOM_MESH,
                fileName=filename, meshScale=scale, **common,
            )
        return -1

    def _origin(self, origin):
        if origin is None:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        position = self._numbers(origin.attrib.get("xyz"), 3, (0.0, 0.0, 0.0))
        rpy = self._numbers(origin.attrib.get("rpy"), 3, (0.0, 0.0, 0.0))
        return position, self.pybullet.getQuaternionFromEuler(rpy)

    @staticmethod
    def _numbers(value, count: int, default):
        if value is None:
            return default
        values = tuple(float(item) for item in value.split())
        if len(values) != count:
            raise ValueError(f"expected {count} numbers, got {value!r}")
        return values

    @staticmethod
    def _decode(value: bytes | str) -> str:
        return value.decode("utf-8") if isinstance(value, bytes) else str(value)

    def _link_pose(self, body_id: int, link_index: int):
        if link_index < 0:
            return self.pybullet.getBasePositionAndOrientation(
                body_id, physicsClientId=self.connection
            )
        state = self.pybullet.getLinkState(
            body_id, link_index, computeForwardKinematics=True,
            physicsClientId=self.connection,
        )
        return state[4], state[5]
