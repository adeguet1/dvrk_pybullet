"""Backend-specific exception hierarchy."""


class PyBulletBackendError(RuntimeError):
    """Base class for actionable PyBullet backend failures."""


class PyBulletDependencyError(PyBulletBackendError):
    """Raised when the PyBullet Python module is unavailable."""


class GStreamerDependencyError(PyBulletBackendError):
    """Raised when the Python GStreamer Unix-FD stack is unavailable."""
