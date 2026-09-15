"""Locate dvrk_model through the ROS 2 ament package index."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from .errors import PyBulletBackendError


def locate_dvrk_model() -> Path:
    """Return the installed ``dvrk_model`` share directory.

    The initial backend intentionally supports only ament-index discovery. A
    missing package normally means the workspace containing ``dvrk_model`` has
    not been built or sourced.
    """
    try:
        root = Path(get_package_share_directory("dvrk_model")).resolve()
    except PackageNotFoundError as error:
        raise PyBulletBackendError(
            "dvrk_model was not found in the ament index; build the workspace "
            "and source its install/setup.bash before starting dvrk_pybullet"
        ) from error

    virtual = root / "urdf" / "Virtual"
    if not virtual.is_dir():
        raise PyBulletBackendError(
            f"the ament-index dvrk_model package has no urdf/Virtual directory: {root}"
        )
    return root
