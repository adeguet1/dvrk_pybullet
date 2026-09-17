"""Load backend-neutral scene objects into a PyBullet world."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

from ament_index_python.packages import get_package_share_directory

from dvrk_simulator_base.scene import SceneObject

from .errors import PyBulletBackendError


@dataclass(frozen=True)
class LoadedSceneObject:
    spec: SceneObject
    body_id: int


def resolve_asset_uri(asset: str) -> Path:
    """Resolve package://<package>/<relative-path> and absolute asset paths."""
    if asset.startswith("package://"):
        package, separator, relative = asset[len("package://") :].partition("/")
        if not package or not separator or not relative:
            raise PyBulletBackendError(f"invalid scene asset URI {asset!r}")
        try:
            path = Path(get_package_share_directory(package)) / relative
        except Exception as error:
            raise PyBulletBackendError(
                f"could not locate package for scene asset {asset!r}"
            ) from error
    else:
        path = Path(asset).expanduser()
        if not path.is_absolute():
            raise PyBulletBackendError(
                f"scene asset must be package:// URI or absolute path: {asset!r}"
            )
    path = path.resolve()
    if not path.is_file():
        raise PyBulletBackendError(f"scene asset does not exist: {path}")
    return path


def load_scene_objects(
    pybullet: Any, objects: Iterable[SceneObject], *, connection: int
) -> dict[str, LoadedSceneObject]:
    """Load configured fixed or dynamic URDF objects into one Bullet world."""
    loaded = {}
    for spec in objects:
        body_id = pybullet.loadURDF(
            str(resolve_asset_uri(spec.asset)),
            basePosition=spec.position,
            baseOrientation=spec.orientation_xyzw,
            useFixedBase=spec.fixed,
            physicsClientId=connection,
        )
        if body_id < 0:
            raise PyBulletBackendError(f"PyBullet could not load scene object {spec.name!r}")
        loaded[spec.name] = LoadedSceneObject(spec, body_id)
    return loaded
