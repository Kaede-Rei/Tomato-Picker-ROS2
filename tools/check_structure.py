#!/usr/bin/env python3
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
TOMATO_PICKER_SRC = SRC / "tomato_picker"
EXPECTED = {
    "interfaces": "tomato_picker_interfaces",
    "perception": "tomato_picker_perception",
    "motion": "tomato_picker_motion",
    "eef": "tomato_picker_eef",
    "task": "tomato_picker_task",
    "bringup": "tomato_picker_bringup",
}


def main() -> int:
    if not TOMATO_PICKER_SRC.is_dir():
        print(f"missing source root: {TOMATO_PICKER_SRC.relative_to(ROOT)}")
        return 1

    expected_directories = {Path(directory) for directory in EXPECTED}
    found_directories = {
        package_xml.parent.relative_to(TOMATO_PICKER_SRC)
        for package_xml in TOMATO_PICKER_SRC.rglob("package.xml")
    }
    if found_directories != expected_directories:
        missing = sorted(str(path) for path in expected_directories - found_directories)
        extra = sorted(str(path) for path in found_directories - expected_directories)
        if missing:
            print(f"missing ROS packages: {missing}")
        if extra:
            print(f"unexpected ROS packages: {extra}")
        return 1

    for directory, package in sorted(EXPECTED.items()):
        package_xml = TOMATO_PICKER_SRC / directory / "package.xml"
        root = ET.parse(package_xml).getroot()
        name = root.findtext("name")
        if name != package:
            print(f"package name mismatch: {package_xml}: {name}")
            return 1

    forbidden = ("piper_sdk", "piper_ros", "piper_noetic")
    for path in TOMATO_PICKER_SRC.rglob("*"):
        if not path.is_file() or path.suffix not in {".cpp", ".hpp", ".h", ".py", ".xml", ".txt"}:
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for token in forbidden:
            if token in text:
                print(f"forbidden legacy dependency '{token}' in {path.relative_to(ROOT)}")
                return 1

    print("structure check passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
