from pathlib import Path

from dvrk_pybullet.urdf_materializer import (
    default_generated_root,
    materialize_virtual_robot,
    materialize_virtual_psm,
)


def test_generated_root_is_workspace_local():
    anchor = Path("/work/ws/src/dvrk/dvrk_pybullet/dvrk_pybullet/module.py")
    assert default_generated_root(anchor) == Path("/work/ws/.generated/pybullet")


def test_virtual_psm1_is_expanded_and_cached(tmp_path):
    first = materialize_virtual_psm(generated_root=tmp_path)
    second = materialize_virtual_psm(generated_root=tmp_path)

    assert first == second
    assert first.urdf_path.is_file()
    assert first.metadata_path.is_file()
    text = first.urdf_path.read_text(encoding="utf-8")
    assert '<robot name="PSM1">' in text
    assert 'joint name="yaw"' in text
    assert "package://dvrk_model/" not in text
    assert 'filename="/' in text


def test_virtual_ecm_is_expanded_and_cached(tmp_path):
    result = materialize_virtual_robot(
        "ECM", endoscope="Si_straight", generated_root=tmp_path
    )
    text = result.urdf_path.read_text(encoding="utf-8")
    assert result.instrument is None
    assert result.endoscope == "Si_straight"
    assert '<robot name="ECM">' in text
    assert 'joint name="insertion"' in text
    assert 'link name="ECM_tip_link"' in text
    assert "package://dvrk_model/" not in text
