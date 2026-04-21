#!/usr/bin/env python3
"""Inspect a PLY file header without loading the full splat file."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class PlyElement:
    name: str
    count: int
    properties: list[str] = field(default_factory=list)


@dataclass
class PlyHeader:
    format_name: str
    version: str
    elements: list[PlyElement]
    comments: list[str]


COMMON_SPLAT_PROPERTIES = {
    "x",
    "y",
    "z",
    "nx",
    "ny",
    "nz",
    "f_dc_0",
    "f_dc_1",
    "f_dc_2",
    "opacity",
    "scale_0",
    "scale_1",
    "scale_2",
    "rot_0",
    "rot_1",
    "rot_2",
    "rot_3",
}


def parse_header(path: Path) -> PlyHeader:
    elements: list[PlyElement] = []
    comments: list[str] = []
    format_name = ""
    version = ""
    current: PlyElement | None = None

    with path.open("rb") as file:
        first = file.readline().decode("ascii", errors="replace").strip()
        if first != "ply":
            raise ValueError(f"{path} does not start with a PLY header")

        for raw_line in file:
            line = raw_line.decode("ascii", errors="replace").strip()
            if line == "end_header":
                break
            if not line:
                continue

            parts = line.split()
            keyword = parts[0]

            if keyword == "format" and len(parts) >= 3:
                format_name = parts[1]
                version = parts[2]
            elif keyword == "comment":
                comments.append(line.removeprefix("comment").strip())
            elif keyword == "element" and len(parts) == 3:
                current = PlyElement(name=parts[1], count=int(parts[2]))
                elements.append(current)
            elif keyword == "property" and current is not None:
                current.properties.append(parts[-1])
        else:
            raise ValueError(f"{path} ended before end_header")

    if not format_name:
        raise ValueError(f"{path} has no PLY format line")

    return PlyHeader(format_name=format_name, version=version, elements=elements, comments=comments)


def print_report(path: Path, header: PlyHeader) -> None:
    print(f"File: {path}")
    print(f"Size: {path.stat().st_size:,} bytes")
    print(f"Format: {header.format_name} {header.version}")
    print()

    for element in header.elements:
        print(f"Element: {element.name}")
        print(f"  Count: {element.count:,}")
        if element.properties:
            print(f"  Properties: {', '.join(element.properties)}")

        if element.name == "vertex":
            found = set(element.properties)
            missing = sorted(COMMON_SPLAT_PROPERTIES - found)
            present = sorted(COMMON_SPLAT_PROPERTIES & found)
            print(f"  Common splat properties present: {len(present)}/{len(COMMON_SPLAT_PROPERTIES)}")
            if missing:
                print(f"  Common splat properties missing: {', '.join(missing)}")
        print()

    if header.comments:
        print("Comments:")
        for comment in header.comments:
            print(f"  - {comment}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect a PLY file header.")
    parser.add_argument("ply_file", type=Path, help="Path to a .ply file")
    args = parser.parse_args()

    if not args.ply_file.is_file():
        parser.error(f"file not found: {args.ply_file}")

    header = parse_header(args.ply_file)
    print_report(args.ply_file, header)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
