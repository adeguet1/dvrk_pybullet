import json
from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]
OPEN_XR = ROOT / "share" / "open-xr"


def test_openxr_video_uses_dvrk_socket_and_declares_stereo_layout():
    config = json.loads((OPEN_XR / "sawOpenXR-pybullet-unixfd.json").read_text())
    assert config["video"] == {
        "type": "side-by-side",
        "gst_input": "@dvrk:pybullet:stereo_source",
    }
    scene = yaml.safe_load(
        (ROOT / "share" / "scenes" / "ECM_PSM1_PSM2_PSM3.yaml").read_text()
    )["scene"]
    assert scene["camera"]["mode"] == "stereo"
    assert (scene["camera"]["width"], scene["camera"]["height"]) == (800, 450)
    assert scene["camera"]["publish_rate_hz"] == 20.0
    assert scene["camera"]["unixfd"]["socket_path"] == config["video"]["gst_input"]


def test_openxr_launch_is_a_rigid_headless_example():
    launch = (ROOT / "launch" / "open_xr.launch.py").read_text()
    assert 'DeclareLaunchArgument(\n                "config"' not in launch
    assert 'DeclareLaunchArgument(\n                "scene"' not in launch
    assert 'DeclareLaunchArgument(\n                "system_config"' not in launch
    assert '"gui",\n                default_value="false"' in launch
    assert "dvrk_system_delay" not in launch
    assert "TimerAction" not in launch
    assert 'open_xr_directory / "pybullet.yaml"' in launch

    config = yaml.safe_load((OPEN_XR / "pybullet.yaml").read_text())
    assert config["renderer"] == "egl"
    assert config["gui"] is False


def test_openxr_system_imports_three_psms_and_ecm_from_ros():
    config = json.loads(
        (OPEN_XR / "system-MTML-MTMR-OpenXR-patient-cart-ROS.json").read_text()
    )
    arms = {arm["name"]: arm for arm in config["arms"]}
    assert set(arms) == {"MTML", "MTMR", "PSM1", "PSM2", "PSM3", "ECM"}
    assert all(arms[name]["skip_ROS_bridge"] for name in ("PSM1", "PSM2", "PSM3", "ECM"))
    teleops = config["consoles"][0]["teleop_PSMs"]
    assert {
        ("MTMR", "PSM1"),
        ("MTML", "PSM2"),
    }.issubset({(entry["MTM"], entry["PSM"]) for entry in teleops})
    components = config["component_manager"]["components"]
    assert components[0]["configure-parameter"] == "sawOpenXR-pybullet-unixfd.json"
