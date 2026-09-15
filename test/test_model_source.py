from dvrk_pybullet.errors import PyBulletBackendError
import dvrk_pybullet.model_source as model_source
import pytest


def test_locate_dvrk_model_uses_ament_index(monkeypatch, tmp_path):
    (tmp_path / "urdf" / "Virtual").mkdir(parents=True)
    monkeypatch.setattr(
        model_source, "get_package_share_directory", lambda package: str(tmp_path)
    )
    assert model_source.locate_dvrk_model() == tmp_path.resolve()


def test_locate_dvrk_model_validates_install(monkeypatch, tmp_path):
    monkeypatch.setattr(
        model_source, "get_package_share_directory", lambda package: str(tmp_path)
    )
    with pytest.raises(PyBulletBackendError, match="urdf/Virtual"):
        model_source.locate_dvrk_model()
