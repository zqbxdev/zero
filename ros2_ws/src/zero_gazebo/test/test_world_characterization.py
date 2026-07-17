from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Final
from xml.etree import ElementTree


PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
SMOKE_WORLD_PATH: Final = PACKAGE_ROOT / "worlds" / "zero_sensor_smoke.sdf"
MODEL_CONFIG_PATH: Final = (
    PACKAGE_ROOT / "models" / "zero_usv" / "model.config"
)
SMOKE_WORLD_SHA256: Final = (
    "36536dccb0d7e2ac90b79a224111a5ffc07bf5eddac690a9884ab211d8a30f56"
)


def test_sensor_smoke_world_bytes_are_preserved() -> None:
    # Given: the characterized sensor smoke world source bytes.
    smoke_world_bytes = SMOKE_WORLD_PATH.read_bytes()

    # When: the source digest is calculated.
    digest = hashlib.sha256(smoke_world_bytes).hexdigest()

    # Then: navigation-world work cannot rewrite the preserved smoke world.
    assert digest == SMOKE_WORLD_SHA256


def test_world_resources_keep_the_existing_directory_install_rule() -> None:
    # Given: the current zero_gazebo CMake resource installation contract.
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    # When: the shared resource install block is inspected.
    install_rule = "DIRECTORY config launch models worlds"

    # Then: every source world remains installed without a file-specific rule.
    assert cmake.count(install_rule) == 1
    assert cmake.count("DESTINATION share/${PROJECT_NAME}") == 1


def test_smoke_world_uses_the_installed_model_resource_convention() -> None:
    # Given: the smoke world include and zero_usv model metadata.
    world = ElementTree.parse(SMOKE_WORLD_PATH).getroot().find("world")
    config = ElementTree.parse(MODEL_CONFIG_PATH).getroot()
    assert world is not None

    # When: model URIs and the configured SDF resource are read.
    model_uris = [uri.text for uri in world.findall("include/uri")]
    model_resource = config.find("sdf")
    assert model_resource is not None

    # Then: source and installed layouts resolve the same model:// resource.
    assert model_uris == ["model://zero_usv"]
    assert config.findtext("name") == "zero_usv"
    assert model_resource.attrib == {"version": "1.8"}
    assert model_resource.text == "model.sdf"
