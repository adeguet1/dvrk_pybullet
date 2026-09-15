"""One PyBullet connection shared by every arm in a configured scene."""

from __future__ import annotations

import importlib.util
import time
from typing import Callable, Mapping

from dvrk_simulator_base.command_mailbox import CommandMailboxes
from dvrk_simulator_base.config import RobotConfig
from dvrk_simulator_base.snapshots import ArmSnapshot

from .backend import load_pybullet
from .camera import CameraOptions, PyBulletCamera
from .errors import PyBulletBackendError
from .runtime import PyBulletRuntime, RuntimeOptions
from .video import UnixFdVideoSink


class PyBulletWorldRuntime:
    def __init__(
        self,
        configs: tuple[RobotConfig, ...],
        options: RuntimeOptions,
        commands: Mapping[str, CommandMailboxes],
        camera_options: CameraOptions | None = None,
        video_sink_factory: Callable[[CameraOptions], object] = UnixFdVideoSink,
    ) -> None:
        if not configs:
            raise ValueError("a PyBullet world requires at least one robot")
        self.options = options
        self.pybullet = load_pybullet()
        self.connection = -1
        self.camera_options = camera_options
        self.camera = None
        self.video_sink = None
        self._video_sink_factory = video_sink_factory
        self._egl_plugin = -1
        self._next_camera_time = 0.0
        self.arms = {
            config.name: PyBulletRuntime(
                config,
                options,
                commands=commands[config.name],
                pybullet_client=self.pybullet,
            )
            for config in configs
        }

    def initialize(self) -> dict[str, ArmSnapshot]:
        mode = self.pybullet.GUI if self.options.gui else self.pybullet.DIRECT
        self.connection = self.pybullet.connect(mode)
        if self.connection < 0:
            raise PyBulletBackendError("PyBullet could not create a world connection")
        try:
            self.pybullet.setGravity(0.0, 0.0, -9.81)
            snapshots = {
                name: arm.initialize(self.connection)
                for name, arm in self.arms.items()
            }
            self._initialize_camera()
            if self.options.gui:
                self.pybullet.resetDebugVisualizerCamera(
                    cameraDistance=0.8,
                    cameraYaw=45.0,
                    cameraPitch=-25.0,
                    cameraTargetPosition=(0.0, 0.0, 0.1),
                )
            return snapshots
        except BaseException:
            self.shutdown()
            raise

    def step(self) -> dict[str, ArmSnapshot]:
        now_ns = time.monotonic_ns()
        now = now_ns * 1e-9
        for arm in self.arms.values():
            arm.prepare_step(now_ns, now)
        self.pybullet.stepSimulation()
        snapshots = {name: arm.finish_step() for name, arm in self.arms.items()}
        self._publish_camera_if_due(snapshots)
        return snapshots

    def _initialize_camera(self) -> None:
        options = self.camera_options
        if options is None or not options.enabled or "ECM" not in self.arms:
            return
        if options.renderer == "tiny":
            renderer = self.pybullet.ER_TINY_RENDERER
        elif self.options.gui:
            renderer = self.pybullet.ER_BULLET_HARDWARE_OPENGL
        else:
            spec = importlib.util.find_spec("eglRenderer")
            if spec is None or not spec.origin:
                raise PyBulletBackendError(
                    "headless EGL renderer is unavailable; install the PyBullet EGL "
                    "plugin or set renderer: tiny in pybullet.yaml"
                )
            self._egl_plugin = self.pybullet.loadPlugin(
                spec.origin, "_eglRendererPlugin", physicsClientId=self.connection
            )
            if self._egl_plugin < 0:
                raise PyBulletBackendError(
                    "PyBullet failed to load its headless EGL renderer; use "
                    "renderer: tiny in pybullet.yaml to diagnose without GPU rendering"
                )
            renderer = self.pybullet.ER_BULLET_HARDWARE_OPENGL
        self.camera = PyBulletCamera(
            self.pybullet, self.connection, options, renderer
        )
        self.video_sink = self._video_sink_factory(options)
        self.video_sink.start()

    def _publish_camera_if_due(
        self, snapshots: Mapping[str, ArmSnapshot]
    ) -> None:
        if self.camera is None or self.video_sink is None:
            return
        snapshot = snapshots["ECM"]
        if snapshot.simulation_time + 1e-12 < self._next_camera_time:
            return
        self.video_sink.push(
            self.camera.capture(snapshot.measured_cp_world, snapshot.simulation_time)
        )
        period = 1.0 / self.camera_options.rate_hz
        while self._next_camera_time <= snapshot.simulation_time + 1e-12:
            self._next_camera_time += period

    def is_connected(self) -> bool:
        return self.connection >= 0 and bool(self.pybullet.isConnected(self.connection))

    def run(self, publish_snapshots, should_continue=None) -> None:
        period = 1.0 / self.options.simulation_rate_hz
        deadline = time.monotonic()
        while self.is_connected() and (
            should_continue is None or should_continue()
        ):
            publish_snapshots(self.step())
            deadline += period
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                deadline = time.monotonic()

    def shutdown(self) -> None:
        if self.video_sink is not None:
            self.video_sink.close()
        self.video_sink = None
        self.camera = None
        for arm in self.arms.values():
            arm.shutdown()
        if (
            self._egl_plugin >= 0
            and self.connection >= 0
            and self.pybullet.isConnected(self.connection)
        ):
            self.pybullet.unloadPlugin(
                self._egl_plugin, physicsClientId=self.connection
            )
        self._egl_plugin = -1
        if self.connection >= 0 and self.pybullet.isConnected(self.connection):
            self.pybullet.disconnect(self.connection)
        self.connection = -1
