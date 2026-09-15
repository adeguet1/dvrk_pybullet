"""PyBullet dependency boundary and backend bootstrap helpers."""

from __future__ import annotations

from types import ModuleType

from .errors import PyBulletDependencyError


def load_pybullet() -> ModuleType:
    """Load PyBullet lazily so package inspection does not require pip setup."""
    try:
        import pybullet
    except ImportError as error:
        raise PyBulletDependencyError(
            "PyBullet is not installed. Activate the documented "
            "system-site-packages virtual environment and run "
            "`python -m pip install pybullet`."
        ) from error
    return pybullet
