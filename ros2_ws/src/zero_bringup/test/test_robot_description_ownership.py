import ast
from pathlib import Path
from xml.etree import ElementTree


BRINGUP_DIR = Path(__file__).resolve().parents[1]
SOURCE_DIR = BRINGUP_DIR.parent
REPOSITORY_DIR = SOURCE_DIR.parents[1]
DESCRIPTION_DIR = SOURCE_DIR / "zero_description"
GAZEBO_DIR = SOURCE_DIR / "zero_gazebo"
TECHNICAL_NOTES_PATH = REPOSITORY_DIR / "docs" / "package-technical-notes.md"
OBSOLETE_XACRO = "zero_usv_sensors" + ".urdf.xacro"


def test_zero_description_owns_the_only_ros_robot_description() -> None:
    # Given: source package metadata and the Gazebo sensor launch.
    launch_source = (
        GAZEBO_DIR / "launch" / "gazebo_sensors.launch.py"
    ).read_text(encoding="utf-8")
    cmake_source = (GAZEBO_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    package_root = ElementTree.parse(GAZEBO_DIR / "package.xml").getroot()
    dependencies = {element.text for element in package_root.findall("exec_depend")}

    # When/Then: Gazebo consumes the installed authoritative URDF without owning another one.
    assert (DESCRIPTION_DIR / "urdf" / "robot.urdf").is_file()
    assert not (GAZEBO_DIR / "urdf" / OBSOLETE_XACRO).exists()
    assert "zero_description" in dependencies
    assert "xacro" not in dependencies
    assert "DIRECTORY config launch models worlds" in cmake_source
    assert "zero_description" in launch_source
    assert '"urdf" / "robot.urdf"' in launch_source
    assert "Command(" not in launch_source
    assert ast.parse(launch_source)


def test_obsolete_gazebo_xacro_has_no_current_source_or_doc_references() -> None:
    # Given: current source and technical notes, excluding tests and historical evidence.
    current_files = [
        path
        for path in SOURCE_DIR.rglob("*")
        if path.is_file()
        and "test" not in path.parts
        and path.suffix in {".py", ".xml", ".txt", ".md", ".xacro", ".sdf", ".urdf"}
    ] + [TECHNICAL_NOTES_PATH]

    # When: references to the obsolete xacro filename are counted.
    reference_count = sum(
        path.read_text(encoding="utf-8", errors="strict").count(OBSOLETE_XACRO)
        for path in current_files
    )

    # Then: no current source or current technical note retains the obsolete path.
    assert reference_count == 0


def test_current_technical_notes_describe_authoritative_sensor_frames() -> None:
    # Given: the current-source technical notes rather than historical evidence.
    technical_notes = TECHNICAL_NOTES_PATH.read_text(encoding="utf-8")

    # When/Then: ownership and aligned sensor poses match the source XML contracts.
    assert "xacro" not in technical_notes
    assert "zero_description/urdf/robot.urdf" in technical_notes
    assert "-0.00015 -0.00040397 0.1277" in technical_notes
    assert "0.15 0 0.13" in technical_notes


def test_simulation_launches_do_not_include_display_only_frames() -> None:
    # Given: all source simulation launch files.
    launch_sources = [
        path.read_text(encoding="utf-8")
        for package in (BRINGUP_DIR, GAZEBO_DIR)
        for path in (package / "launch").glob("*.launch.py")
    ]

    # When/Then: display launch and its temporary map transform remain uncomposed.
    assert launch_sources
    assert all("display.launch.py" not in source for source in launch_sources)
    assert all("static_map_to_zero_base_link" not in source for source in launch_sources)
