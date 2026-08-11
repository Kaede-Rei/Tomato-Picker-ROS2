#!/usr/bin/env python3
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
TOMATO = ROOT / "src" / "tomato_picker"
EXPECTED = {
    "interfaces": "tomato_picker_interfaces",
    "perception": "tomato_picker_perception",
    "motion": "tomato_picker_motion",
    "eef": "tomato_picker_eef",
    "task": "tomato_picker_task",
    "bringup": "tomato_picker_bringup",
}


def main() -> int:
    found = {
        path.parent.relative_to(TOMATO)
        for path in TOMATO.rglob("package.xml")
    }
    expected = {Path(name) for name in EXPECTED}
    if found != expected:
        print(f"package layout mismatch: expected={sorted(map(str, expected))} found={sorted(map(str, found))}")
        return 1

    for directory, package in EXPECTED.items():
        name = ET.parse(TOMATO / directory / "package.xml").getroot().findtext("name")
        if name != package:
            print(f"package name mismatch: {directory}: {name}")
            return 1

    launch_files = sorted((TOMATO / "bringup" / "launch").glob("*.launch.py"))
    if [path.name for path in launch_files] != ["bringup.launch.py"]:
        print(f"bringup must expose only bringup.launch.py: {[path.name for path in launch_files]}")
        return 1

    launch_text = launch_files[0].read_text(encoding="utf-8")
    required = [
        "serial_arm_ros2_control",
        "moveit.launch.py",
        "eef_controller",
        "wait_for_arm_ready",
        "wait_for_eef_ready",
    ]
    forbidden = ["ros2_control_node", "hardware_spawner", "MoveItConfigsBuilder", "profile_utils"]
    for token in required:
        if token not in launch_text:
            print(f"bringup missing required composition token: {token}")
            return 1
    for token in forbidden:
        if token in launch_text:
            print(f"bringup reimplements SerialArm responsibility: {token}")
            return 1

    repos = (ROOT / "repos" / "serial_arm.repos").read_text(encoding="utf-8")
    if "version: main" not in repos:
        print("SerialArm-Core must track main")
        return 1

    for package in ("interfaces", "perception", "motion", "task"):
        package_root = TOMATO / package
        for path in package_root.rglob("*"):
            if not path.is_file() or path.suffix not in {".cpp", ".hpp", ".h", ".xml", ".yaml", ".py"}:
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            if "serial_arm" in text.lower() or "damiao" in text.lower():
                print(f"upper-layer package depends on arm backend: {path.relative_to(ROOT)}")
                return 1

    print("structure check passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
