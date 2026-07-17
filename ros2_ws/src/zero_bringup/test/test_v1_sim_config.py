from __future__ import annotations

from copy import deepcopy
from math import isfinite
from pathlib import Path
from typing import Literal, NoReturn

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parent
CONFIG_PATH = PACKAGE_ROOT / "config" / "v1_sim.yaml"
TIMEOUT_SECONDS = 0.5
ParameterValue = bool | float | int
Parameters = dict[str, ParameterValue]
Contract = dict[str, dict[str, Parameters]]
Mutation = Literal[
    "missing_section",
    "renamed_max_key",
    "non_finite_geometry",
    "invalid_sign",
    "missing_timeout",
]
REQUIRED_PARAMETERS: dict[str, Parameters] = {
    "robot_state_publisher": {"use_sim_time": True},
    "twist_to_motor_command": {
        "use_sim_time": True,
        "max_rpm": 300.0,
        "max_linear_mps": 1.0,
        "max_angular_radps": 1.0,
        "track_width_m": 0.407,
        "wheel_radius_m": 0.05,
        "left_motor_sign": 1,
        "right_motor_sign": 1,
        "command_timeout_seconds": TIMEOUT_SECONDS,
    },
    "command_guard": {
        "use_sim_time": True,
        "max_rpm": 300.0,
        "command_timeout_seconds": TIMEOUT_SECONDS,
        "status_timeout_seconds": TIMEOUT_SECONDS,
        "motor_state_timeout_seconds": TIMEOUT_SECONDS,
    },
    "fake_motor_controller": {
        "use_sim_time": True,
        "max_rpm": 300.0,
        "command_timeout_seconds": TIMEOUT_SECONDS,
    },
    "gazebo_drive_adapter": {
        "use_sim_time": True,
        "max_rpm": 300.0,
        "max_linear_mps": 1.0,
        "max_angular_radps": 1.0,
        "track_width_m": 0.407,
        "wheel_radius_m": 0.05,
        "left_motor_sign": 1,
        "right_motor_sign": 1,
        "motor_state_timeout_seconds": TIMEOUT_SECONDS,
    },
    "odom_to_tf_relay": {"use_sim_time": True},
    "simulation_safety_initializer": {
        "use_sim_time": True,
        "status_timeout_seconds": TIMEOUT_SECONDS,
        "motor_state_timeout_seconds": TIMEOUT_SECONDS,
    },
}
POSITIVE_FINITE_KEYS = {
    "max_rpm",
    "max_linear_mps",
    "max_angular_radps",
    "track_width_m",
    "wheel_radius_m",
    "command_timeout_seconds",
    "status_timeout_seconds",
    "motor_state_timeout_seconds",
}
SIGN_KEYS = {"left_motor_sign", "right_motor_sign"}


def assert_never(value: NoReturn) -> NoReturn:
    raise AssertionError(f"Unhandled mutation: {value!r}")


def load_contract(path: Path = CONFIG_PATH) -> Contract:
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def validate_contract(contract: Contract) -> None:
    assert isinstance(contract, dict)
    assert set(contract) == set(REQUIRED_PARAMETERS)
    for node_name, expected_parameters in REQUIRED_PARAMETERS.items():
        node = contract[node_name]
        assert set(node) == {"ros__parameters"}
        parameters = node["ros__parameters"]
        assert parameters == expected_parameters
        assert "max_command_rpm" not in parameters
        for key in POSITIVE_FINITE_KEYS & parameters.keys():
            value = parameters[key]
            assert isinstance(value, (int, float)) and not isinstance(value, bool)
            assert isfinite(value) and value > 0.0
        for key in SIGN_KEYS & parameters.keys():
            assert parameters[key] in {-1, 1}


def find_v1_configs(root: Path) -> list[Path]:
    return sorted(root.rglob("v1_sim.yaml"))


def require_single_v1_config(root: Path) -> Path:
    candidates = find_v1_configs(root)
    assert len(candidates) == 1
    return candidates[0]


def test_v1_sim_config_is_the_only_canonical_copy() -> None:
    # Given: every source package beneath the ROS workspace.
    resolved_config = require_single_v1_config(SOURCE_ROOT)

    # When/Then: exactly the bringup-owned V1 contract is present.
    assert resolved_config == CONFIG_PATH


def test_v1_sim_config_matches_the_exact_contract() -> None:
    # Given: the canonical YAML parsed through the production-safe loader.
    contract = load_contract()

    # When/Then: every section and resolved value matches V1 exactly.
    validate_contract(contract)


def test_launch_files_do_not_copy_v1_parameters() -> None:
    # Given: all bringup launch source files.
    launch_source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in sorted((PACKAGE_ROOT / "launch").glob("*.py"))
    )

    # When/Then: launch composition owns paths, not copies of V1 tuning keys.
    for key in POSITIVE_FINITE_KEYS | SIGN_KEYS | {"max_command_rpm"}:
        assert key not in launch_source


def assert_mutation_rejected(mutation: Mutation) -> None:
    # Given: one isolated malformed mutation of the expected contract.
    contract: Contract = {
        node: {"ros__parameters": deepcopy(parameters)}
        for node, parameters in REQUIRED_PARAMETERS.items()
    }
    match mutation:
        case "missing_section":
            del contract["odom_to_tf_relay"]
        case "renamed_max_key":
            parameters = contract["command_guard"]["ros__parameters"]
            parameters["max_command_rpm"] = parameters.pop("max_rpm")
        case "non_finite_geometry":
            contract["twist_to_motor_command"]["ros__parameters"]["track_width_m"] = float("nan")
        case "invalid_sign":
            contract["gazebo_drive_adapter"]["ros__parameters"]["left_motor_sign"] = 0
        case "missing_timeout":
            del contract["fake_motor_controller"]["ros__parameters"]["command_timeout_seconds"]
        case unreachable:
            assert_never(unreachable)

    # When/Then: schema validation fails closed.
    rejected = False
    try:
        validate_contract(contract)
    except AssertionError:
        rejected = True
    assert rejected


def test_v1_sim_config_rejects_missing_section() -> None:
    assert_mutation_rejected("missing_section")


def test_v1_sim_config_rejects_renamed_max_key() -> None:
    assert_mutation_rejected("renamed_max_key")


def test_v1_sim_config_rejects_non_finite_geometry() -> None:
    assert_mutation_rejected("non_finite_geometry")


def test_v1_sim_config_rejects_invalid_sign() -> None:
    assert_mutation_rejected("invalid_sign")


def test_v1_sim_config_rejects_missing_timeout() -> None:
    assert_mutation_rejected("missing_timeout")


def test_duplicate_v1_config_fixture_is_rejected(tmp_path: Path) -> None:
    # Given: two package-like fixtures claiming the canonical filename.
    for package_name in ("zero_bringup", "zero_duplicate"):
        config_dir = tmp_path / package_name / "config"
        config_dir.mkdir(parents=True)
        _ = (config_dir / "v1_sim.yaml").write_text("{}\n", encoding="utf-8")

    # When/Then: the same uniqueness resolver rejects the fixture set.
    rejected = False
    try:
        require_single_v1_config(tmp_path)
    except AssertionError:
        rejected = True
    assert rejected
