from collections import Counter
from pathlib import Path
import re
import shlex
from typing import Final


REPOSITORY_ROOT = Path(__file__).parents[4]
DOCKERFILE = REPOSITORY_ROOT / "Dockerfile"
REQUIRED_FULL_PACKAGES: Final = (
    "ros-humble-navigation2",
    "ros-humble-nav2-bringup",
    "ros-humble-slam-toolbox",
    "ros-humble-teleop-twist-keyboard",
    "ros-humble-ros-gz",
    "ros-humble-ros-gz-bridge",
    "ros-humble-xacro",
    "ros-humble-rviz2",
)
EXCLUDED_PACKAGE_MARKERS: Final = (
    "gps",
    "ekf",
    "sros2",
    "robot-localization",
    "water",
    "real-hardware",
)


def _stage_text(dockerfile: str, stage: str) -> str:
    match = re.search(
        rf"^FROM\s+\S+\s+AS\s+{re.escape(stage)}\s*$(?P<body>.*?)(?=^FROM\s|\Z)",
        dockerfile,
        flags=re.MULTILINE | re.DOTALL,
    )
    assert match is not None, f"missing Docker stage: {stage}"
    return match.group("body")


def _package_counts(stage: str) -> Counter[str]:
    run_commands = (
        match.group("command")
        for match in re.finditer(
            r"^RUN[ \t]+(?P<command>(?:[^\n]*\\\n)*[^\n]*)",
            stage,
            flags=re.MULTILINE,
        )
    )
    install_arguments = (
        match.group("arguments")
        for command in run_commands
        for match in re.finditer(
            r"(?:^|&&|;)\s*apt-get\s+install\b(?P<arguments>.*?)(?=\s*(?:&&|;|$))",
            command.replace("\\\n", " "),
        )
    )
    return Counter(
        token
        for arguments in install_arguments
        for token in shlex.split(arguments)
        if re.fullmatch(r"[a-z0-9][a-z0-9+.-]+", token)
    )


def _full_package_contract(full_stage: str) -> tuple[dict[str, int], set[str]]:
    package_counts = _package_counts(full_stage)
    required_counts = {
        package: package_counts[package] for package in REQUIRED_FULL_PACKAGES
    }
    excluded_packages = {
        package
        for package in package_counts
        if any(marker in package for marker in EXCLUDED_PACKAGE_MARKERS)
    }
    return required_counts, excluded_packages


def _apt_install_stage(packages: tuple[str, ...]) -> str:
    return (
        "RUN apt-get install -y --no-install-recommends \\\n"
        + "".join(f"    {package} \\\n" for package in packages[:-1])
        + f"    {packages[-1]} && \\\n"
        + "    apt-get clean\n"
    )


def test_full_stage_preserves_existing_simulation_packages() -> None:
    dockerfile = DOCKERFILE.read_text(encoding="utf-8")

    full_stage = _stage_text(dockerfile, "full")

    assert "FROM dev AS full" in dockerfile
    package_counts = _package_counts(full_stage)
    assert package_counts["ros-humble-desktop"] == 1
    assert package_counts["gz-fortress"] == 1
    assert package_counts["ros-humble-ros-gz"] == 1
    assert package_counts["ros-humble-ros-gz-bridge"] == 1


def test_full_package_contract_detects_duplicate_required_package() -> None:
    duplicate_stage = _apt_install_stage(
        (*REQUIRED_FULL_PACKAGES, "ros-humble-navigation2")
    )

    required_counts, _ = _full_package_contract(duplicate_stage)

    assert required_counts["ros-humble-navigation2"] == 2


def test_full_package_contract_detects_excluded_package() -> None:
    excluded_stage = _apt_install_stage(
        (*REQUIRED_FULL_PACKAGES, "ros-humble-robot-localization")
    )

    _, excluded_packages = _full_package_contract(excluded_stage)

    assert excluded_packages == {"ros-humble-robot-localization"}


def test_full_package_contract_ignores_required_packages_in_env_continuation() -> None:
    env_only_stage = (
        "RUN apt-get install -y --no-install-recommends ca-certificates\n"
        + "ENV REQUIRED_PACKAGE_NAMES=\"placeholder \\\n"
        + "".join(
            f"    {package} \\\n" for package in REQUIRED_FULL_PACKAGES
        )
        + "    end\"\n"
    )

    required_counts, _ = _full_package_contract(env_only_stage)

    assert required_counts == dict.fromkeys(REQUIRED_FULL_PACKAGES, 0)


def test_full_stage_has_required_packages_and_excludes_out_of_scope_dependencies() -> None:
    dockerfile = DOCKERFILE.read_text(encoding="utf-8")

    full_stage = _stage_text(dockerfile, "full")
    required_counts, excluded_packages = _full_package_contract(full_stage)

    assert required_counts == dict.fromkeys(REQUIRED_FULL_PACKAGES, 1)
    assert excluded_packages == set()
