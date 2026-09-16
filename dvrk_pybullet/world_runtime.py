"""One PyBullet connection shared by every arm in a configured scene."""

from __future__ import annotations

import time
from typing import Mapping

import numpy as np

from dvrk_simulator_base.command_mailbox import CommandMailboxes
from dvrk_simulator_base.config import RobotConfig
from dvrk_simulator_base.snapshots import ArmSnapshot

from .backend import load_pybullet
from .camera_worker import CameraWorker
from .errors import PyBulletBackendError
from .runtime import PyBulletRuntime, RuntimeOptions


class PyBulletWorldRuntime:
    def __init__(
        self,
        configs: tuple[RobotConfig, ...],
        options: RuntimeOptions,
        commands: Mapping[str, CommandMailboxes],
        camera_options=None,
    ) -> None:
        if not configs:
            raise ValueError("a PyBullet world requires at least one robot")
        self.options = options
        self.pybullet = load_pybullet()
        self.connection = -1
        self.camera_options = camera_options
        self.camera_worker = None
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
            self._start_camera_worker(snapshots)
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

    def _start_camera_worker(self, snapshots: Mapping[str, ArmSnapshot]) -> None:
        if (
            self.camera_options is None
            or not self.camera_options.enabled
            or "ECM" not in self.arms
        ):
            return
        self.camera_worker = CameraWorker(self.arms, self.camera_options)
        self.camera_worker.start(self._camera_state(snapshots))

    def _camera_state(
        self, snapshots: Mapping[str, ArmSnapshot]
    ) -> tuple[np.ndarray, ...]:
        joint_positions = []
        for arm in self.arms.values():
            count = self.pybullet.getNumJoints(arm.robot.body_id)
            states = self.pybullet.getJointStates(
                arm.robot.body_id, tuple(range(count)),
                physicsClientId=self.connection,
            )
            joint_positions.append(
                np.asarray([state[0] for state in states], dtype=float)
            )
        camera_pose = snapshots["ECM"].measured_cp_world
        return (
            *joint_positions,
            np.asarray(camera_pose.position, dtype=float),
            np.asarray(camera_pose.orientation, dtype=float).reshape(9),
        )

    def step(self) -> dict[str, ArmSnapshot]:
        now_ns = time.monotonic_ns()
        now = now_ns * 1e-9
        for arm in self.arms.values():
            arm.prepare_step(now_ns, now)
        self.pybullet.stepSimulation()
        snapshots = {name: arm.finish_step() for name, arm in self.arms.items()}
        if self.camera_worker is not None:
            self.camera_worker.submit(self._camera_state(snapshots))
            self.camera_worker.check()
        return snapshots

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
        if self.camera_worker is not None:
            self.camera_worker.close()
        self.camera_worker = None
        for arm in self.arms.values():
            arm.shutdown()
        if self.connection >= 0 and self.pybullet.isConnected(self.connection):
            self.pybullet.disconnect(self.connection)
        self.connection = -1
