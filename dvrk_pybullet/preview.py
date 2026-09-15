"""Open a PyBullet GUI containing one dVRK Virtual PSM."""

from __future__ import annotations

import argparse
import sys
import time

from .backend import load_pybullet
from .configuration import load_installed_robot_config
from .errors import PyBulletDependencyError
from .robot import load_robot, reset_joint_positions
from .urdf_materializer import materialize_virtual_psm, SUPPORTED_PSMS


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=SUPPORTED_PSMS, default="PSM1")
    parser.add_argument("--instrument", default="420006")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="seconds to keep the GUI open; zero waits until Ctrl-C",
    )
    args = parser.parse_args(argv)
    if args.duration < 0.0:
        parser.error("--duration cannot be negative")

    try:
        pybullet = load_pybullet()
    except PyBulletDependencyError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    connection = pybullet.connect(pybullet.GUI)
    if connection < 0:
        raise RuntimeError("PyBullet could not create a GUI connection")
    try:
        config = load_installed_robot_config(args.model, args.instrument)
        artifact = materialize_virtual_psm(args.model, args.instrument)
        robot = load_robot(
            pybullet,
            artifact.urdf_path,
            (joint.name for joint in config.joints),
            base_position=config.base_position,
            base_orientation_xyzw=config.base_orientation_xyzw,
        )
        reset_joint_positions(pybullet, robot, config.home_position)
        pybullet.resetDebugVisualizerCamera(
            cameraDistance=0.65,
            cameraYaw=45.0,
            cameraPitch=-25.0,
            cameraTargetPosition=(0.0, 0.0, 0.12),
        )
        print(f"Loaded {args.model} ({args.instrument}) from {artifact.urdf_path}")
        print("Controlled joints:", ", ".join(joint.name for joint in config.joints))
        print("Close the PyBullet window or press Ctrl-C to exit.")

        deadline = time.monotonic() + args.duration if args.duration else None
        while pybullet.isConnected(connection):
            if deadline is not None and time.monotonic() >= deadline:
                break
            pybullet.stepSimulation()
            time.sleep(1.0 / 120.0)
    except KeyboardInterrupt:
        pass
    finally:
        if pybullet.isConnected(connection):
            pybullet.disconnect(connection)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
