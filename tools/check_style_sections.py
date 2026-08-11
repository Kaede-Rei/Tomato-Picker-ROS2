#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
TOMATO_PICKER_SRC = ROOT / "src" / "tomato_picker"
HEADER_SECTIONS = [
    "// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //",
    "// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //",
    "// ! ========================= 模 版 方 法 实 现 ========================= ! //",
]
SOURCE_SECTIONS = [
    "// ! ========================= 宏 定 义 ========================= ! //",
    "// ! ========================= 接 口 变 量 ========================= ! //",
    "// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //",
    "// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //",
    "// ! ========================= 私 有 类 方 法 实 现 ========================= ! //",
]


def check_sections(path: Path, sections: list[str]) -> list[str]:
    text = path.read_text(encoding="utf-8")
    errors = []
    positions = []
    for section in sections:
        count = text.count(section)
        if count != 1:
            errors.append(f"expected one '{section}', found {count}")
        positions.append(text.find(section))
    if all(position >= 0 for position in positions) and positions != sorted(positions):
        errors.append("section markers are out of order")
    return errors


def main() -> int:
    if not TOMATO_PICKER_SRC.is_dir():
        print(f"missing source root: {TOMATO_PICKER_SRC.relative_to(ROOT)}")
        return 1

    failed = False
    production_headers = [
        path for path in TOMATO_PICKER_SRC.rglob("*.hpp")
        if not {"test", "tests"}.intersection(path.relative_to(TOMATO_PICKER_SRC).parts)
    ]
    production_sources = [
        path for path in TOMATO_PICKER_SRC.rglob("*.cpp")
        if not {"test", "tests"}.intersection(path.relative_to(TOMATO_PICKER_SRC).parts)
    ]

    for path, sections in [
        *((path, HEADER_SECTIONS) for path in production_headers),
        *((path, SOURCE_SECTIONS) for path in production_sources),
    ]:
        for error in check_sections(path, sections):
            print(f"{path.relative_to(ROOT)}: {error}")
            failed = True
    if failed:
        return 1
    print(
        "style section check passed: "
        f"{len(production_headers)} headers, {len(production_sources)} sources"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
