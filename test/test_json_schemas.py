import json
import os
from pathlib import Path

import pytest
from jsonschema import Draft7Validator, Draft202012Validator, RefResolver


ROOT = Path(__file__).parents[1]
SHARE = ROOT / "share"
SYSTEM_SCHEMA_ID = "dvrk-system.schema.json"


def _dvrk_schema_directory() -> Path | None:
    configured = os.environ.get("DVRK_SYSTEM_SCHEMA_DIR")
    candidates = (
        Path(configured) if configured else None,
        ROOT.parents[1]
        / "cisst-saw"
        / "sawIntuitiveResearchKit"
        / "share"
        / "schemas",
    )
    for candidate in candidates:
        if candidate and (candidate / SYSTEM_SCHEMA_ID).is_file():
            return candidate
    return None


def _dvrk_system_validator(schema_directory: Path) -> Draft7Validator:
    schema = json.loads((schema_directory / SYSTEM_SCHEMA_ID).read_text())

    class SchemaDirectoryResolver(RefResolver):
        # The dVRK schemas intentionally use their filenames as $ref values;
        # resolve them directly from the schema directory, including files
        # whose filename capitalization differs from their $id.
        def _find_in_subschemas(self, url):
            return None

        def resolve_from_url(self, url):
            return json.loads((schema_directory / Path(url).name).read_text())

    resolver = SchemaDirectoryResolver(SYSTEM_SCHEMA_ID, schema)
    return Draft7Validator(schema, resolver=resolver)


def test_local_json_configurations_validate_against_their_schema():
    for config_path in SHARE.rglob("*.json"):
        if "schemas" in config_path.parts:
            continue
        config = json.loads(config_path.read_text())
        schema_reference = config.get("$schema")
        if not schema_reference:
            continue
        schema_path = (config_path.parent / schema_reference).resolve()
        assert schema_path.is_file(), f"schema not found for {config_path.relative_to(ROOT)}"
        schema = json.loads(schema_path.read_text())
        Draft202012Validator.check_schema(schema)
        errors = sorted(
            Draft202012Validator(schema).iter_errors(config), key=lambda error: str(error.path)
        )
        assert not errors, "\n".join(
            f"{config_path.relative_to(ROOT)}: {error.message}" for error in errors
        )


def test_system_config_uses_the_authoritative_dvrk_schema():
    schema_directory = _dvrk_schema_directory()
    if schema_directory is None:
        pytest.skip(
            "set DVRK_SYSTEM_SCHEMA_DIR to sawIntuitiveResearchKit/share/schemas "
            "to validate the dVRK system configuration"
        )
    config_path = SHARE / "open-xr" / "system-MTML-MTMR-OpenXR-patient-cart-ROS.json"
    config = json.loads(config_path.read_text())
    assert config["$id"] == SYSTEM_SCHEMA_ID
    errors = sorted(
        _dvrk_system_validator(schema_directory).iter_errors(config),
        key=lambda error: str(error.path),
    )
    assert not errors, "\n".join(
        f"{config_path.relative_to(ROOT)}: {error.json_path}: {error.message}"
        for error in errors
    )
