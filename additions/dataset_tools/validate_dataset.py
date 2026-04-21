#!/usr/bin/env python3
"""Validate common dataset layout issues before using Brush."""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".webp", ".tif", ".tiff", ".exr"}
KNOWN_METADATA = {
    "transforms.json",
    "sparse/0/cameras.bin",
    "sparse/0/images.bin",
    "sparse/0/points3D.bin",
    "sparse/0/cameras.txt",
    "sparse/0/images.txt",
    "sparse/0/points3D.txt",
}


def find_images(root: Path) -> list[Path]:
    return sorted(path for path in root.rglob("*") if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS)


def find_empty_dirs(root: Path) -> list[Path]:
    empty_dirs: list[Path] = []
    for path in root.rglob("*"):
        if path.is_dir() and not any(path.iterdir()):
            empty_dirs.append(path)
    return sorted(empty_dirs)


def relative(path: Path, root: Path) -> str:
    return str(path.relative_to(root)).replace("\\", "/")


def validate(root: Path) -> int:
    issues = 0

    print(f"Dataset: {root}")
    print()

    images = find_images(root)
    print(f"Images found: {len(images)}")
    if not images:
        print("ERROR: No image files found.")
        issues += 1
    else:
        by_extension = Counter(path.suffix.lower() for path in images)
        print("Image extensions:")
        for extension, count in sorted(by_extension.items()):
            print(f"  {extension}: {count}")

    lower_names = Counter(path.name.lower() for path in images)
    duplicates = sorted(name for name, count in lower_names.items() if count > 1)
    if duplicates:
        print()
        print("ERROR: Duplicate image filenames when compared case-insensitively:")
        for name in duplicates:
            print(f"  {name}")
        issues += len(duplicates)

    present_metadata = sorted(item for item in KNOWN_METADATA if (root / item).exists())
    print()
    if present_metadata:
        print("Known metadata found:")
        for item in present_metadata:
            print(f"  {item}")
    else:
        print("WARNING: No common Nerfstudio or COLMAP metadata files found.")

    empty_dirs = find_empty_dirs(root)
    if empty_dirs:
        print()
        print("Empty directories:")
        for path in empty_dirs[:20]:
            print(f"  {relative(path, root)}")
        if len(empty_dirs) > 20:
            print(f"  ... {len(empty_dirs) - 20} more")

    print()
    if issues:
        print(f"Validation finished with {issues} issue(s).")
        return 1

    print("Validation finished without blocking issues.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate a dataset folder for common Brush input issues.")
    parser.add_argument("dataset", type=Path, help="Path to a dataset folder")
    args = parser.parse_args()

    if not args.dataset.is_dir():
        parser.error(f"dataset folder not found: {args.dataset}")

    return validate(args.dataset)


if __name__ == "__main__":
    raise SystemExit(main())
