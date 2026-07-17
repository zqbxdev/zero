from pathlib import Path
from typing import Final


REPOSITORY_ROOT: Final = Path(__file__).resolve().parents[4]
PROJECT_STRUCTURE: Final = REPOSITORY_ROOT / "docs" / "project-structure.md"
HISTORICAL_HEADING: Final = "## 历史八包基线证据，原文保留"
HISTORICAL_SENTENCE: Final = (
    "21 个 zero_navigation ROS-free Python tests 包含在本次仓库级 55/55 passed 结果中，"
    "八包 clean build 通过；没有执行 launch、ROS graph、Gazebo 或端到端 smoke，"
    "ROS 运行时验收仍需后续完成。"
)


def test_project_structure_preserves_historical_navigation_evidence() -> None:
    # Given: the current project structure document.
    document = PROJECT_STRUCTURE.read_text(encoding="utf-8")

    # When: the historical baseline section is located.
    heading_index = document.index(HISTORICAL_HEADING)
    sentence_index = document.index(HISTORICAL_SENTENCE)

    # Then: the exact evidence remains under its historical label.
    assert heading_index < sentence_index
