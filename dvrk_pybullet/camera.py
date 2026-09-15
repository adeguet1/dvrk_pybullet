"""ECM optical camera rendering on the PyBullet owner thread."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np

from dvrk_simulator_base.types import Pose


@dataclass(frozen=True)
class CameraOptions:
    enabled: bool = True
    renderer: str = "egl"
    socket_path: Path = Path("/tmp/dvrk-pybullet-ECM.sock")
    width: int = 1280
    height: int = 720
    rate_hz: float = 30.0
    horizontal_fov_degrees: float = 60.0
    near_m: float = 0.01
    far_m: float = 10.0

    def __post_init__(self) -> None:
        renderer = str(self.renderer).lower()
        path = Path(self.socket_path).expanduser()
        if renderer not in {"egl", "tiny"}:
            raise ValueError("renderer must be 'egl' or 'tiny'")
        if not path.is_absolute():
            raise ValueError("camera socket path must be absolute")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera width and height must be positive")
        if not np.isfinite(self.rate_hz) or self.rate_hz <= 0.0:
            raise ValueError("camera rate must be finite and positive")
        if not 0.0 < self.horizontal_fov_degrees < 180.0:
            raise ValueError("camera horizontal FOV must be between 0 and 180 degrees")
        if self.near_m <= 0.0 or self.far_m <= self.near_m:
            raise ValueError("camera clipping planes must satisfy 0 < near < far")
        object.__setattr__(self, "renderer", renderer)
        object.__setattr__(self, "socket_path", path)

    @property
    def vertical_fov_degrees(self) -> float:
        horizontal = math.radians(self.horizontal_fov_degrees)
        vertical = 2.0 * math.atan(math.tan(horizontal / 2.0) / (self.width / self.height))
        return math.degrees(vertical)

    @classmethod
    def from_scene(cls, camera) -> "CameraOptions":
        settings = camera.as_dict()
        if camera.mode not in {"off", "mono"}:
            raise ValueError("PyBullet currently supports off or mono scene cameras")
        encoding = str(settings.get("encoding", "rgba8")).lower()
        if encoding != "rgba8":
            raise ValueError("PyBullet Unix-FD camera encoding must be rgba8")
        transports = settings.get("transports", ["unixfd"])
        if not isinstance(transports, list):
            raise ValueError("scene.camera.transports must be a list")
        unixfd = settings.get("unixfd", {}) or {}
        if not isinstance(unixfd, dict):
            raise ValueError("scene.camera.unixfd must be a mapping")
        return cls(
            enabled=camera.mode != "off" and "unixfd" in transports,
            renderer=str(settings.get("renderer", "egl")),
            socket_path=Path(
                str(unixfd.get("socket_path", "/tmp/dvrk-pybullet-ECM.sock"))
            ),
            width=int(settings.get("width", 1280)),
            height=int(settings.get("height", 720)),
            rate_hz=float(settings.get("publish_rate_hz", 30.0)),
            horizontal_fov_degrees=float(settings.get("horizontal_fov_deg", 60.0)),
            near_m=float(settings.get("near_clip_m", 0.005)),
            far_m=float(settings.get("far_clip_m", 10.0)),
        )


@dataclass(frozen=True)
class VideoFrame:
    rgba: np.ndarray
    simulation_time: float
    sequence: int


def view_vectors(optical_pose: Pose) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return PyBullet eye, target, and up vectors for ECM optical +X/+Z axes."""
    eye = np.asarray(optical_pose.position, dtype=float)
    target = eye + optical_pose.orientation[:, 0]
    up = optical_pose.orientation[:, 2]
    return eye, target, up


class PyBulletCamera:
    def __init__(self, pybullet, connection: int, options: CameraOptions, renderer: int):
        self.pybullet = pybullet
        self.connection = connection
        self.options = options
        self.renderer = renderer
        self.sequence = 0
        self._projection = pybullet.computeProjectionMatrixFOV(
            fov=options.vertical_fov_degrees,
            aspect=options.width / options.height,
            nearVal=options.near_m,
            farVal=options.far_m,
        )

    def capture(self, optical_pose: Pose, simulation_time: float) -> VideoFrame:
        eye, target, up = view_vectors(optical_pose)
        view = self.pybullet.computeViewMatrix(eye, target, up)
        result = self.pybullet.getCameraImage(
            width=self.options.width,
            height=self.options.height,
            viewMatrix=view,
            projectionMatrix=self._projection,
            renderer=self.renderer,
            physicsClientId=self.connection,
        )
        rgba = np.asarray(result[2], dtype=np.uint8).reshape(
            self.options.height, self.options.width, 4
        )
        rgba = np.ascontiguousarray(rgba)
        frame = VideoFrame(rgba, float(simulation_time), self.sequence)
        self.sequence += 1
        return frame
