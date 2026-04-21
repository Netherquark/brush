#!/usr/bin/env python3
"""Validate image masks against an image directory."""

from __future__ import annotations

import argparse
from pathlib import Path


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".webp", ".tif", ".tiff", ".exr"}
MASK_EXTENSIONS = {".png", ".jpg", ".jpeg", ".webp", ".tif", ".tiff"}


try:
    from PIL import Image
except ImportError:  # pragma: no cover - depends on local environment
    Image = None


def collect_by_stem(root: Path, extensions: set[str]) -> dict[str, Path]:
    files: dict[str, Path] = {}
    for path in sorted(root.rglob("*")):
        if path.is_file() and path.suffix.lower() in extensions:
            files.setdefault(path.stem, path)
    return files


def image_size(path: Path) -> tuple[int, int] | None:
    if Image is None:
        return None
    with Image.open(path) as image:
        return image.size


def validate(images_dir: Path, masks_dir: Path) -> int:
    images = collect_by_stem(images_dir, IMAGE_EXTENSIONS)
    masks = collect_by_stem(masks_dir, MASK_EXTENSIONS)
    issues = 0

    print(f"Images: {images_dir}")
    print(f"Masks: {masks_dir}")
    print(f"Image count: {len(images)}")
    print(f"Mask count: {len(masks)}")
    print()

    missing_masks = sorted(set(images) - set(masks))
    extra_masks = sorted(set(masks) - set(images))

    if missing_masks:
        print("Missing masks:")
        for stem in missing_masks[:50]:
            print(f"  {stem}")
        if len(missing_masks) > 50:
            print(f"  ... {len(missing_masks) - 50} more")
        issues += len(missing_masks)
        print()

    if extra_masks:
        print("Masks without matching images:")
        for stem in extra_masks[:50]:
            print(f"  {stem}")
        if len(extra_masks) > 50:
            print(f"  ... {len(extra_masks) - 50} more")
        issues += len(extra_masks)
        print()

    if Image is None:
        print("Dimension check skipped because Pillow is not installed.")
    else:
        size_mismatches = []
        for stem in sorted(set(images) & set(masks)):
            image_dimensions = image_size(images[stem])
            mask_dimensions = image_size(masks[stem])
            if image_dimensions != mask_dimensions:
                size_mismatches.append((stem, image_dimensions, mask_dimensions))

        if size_mismatches:
            print("Dimension mismatches:")
            for stem, image_dimensions, mask_dimensions in size_mismatches[:50]:
                print(f"  {stem}: image={image_dimensions}, mask={mask_dimensions}")
            if len(size_mismatches) > 50:
                print(f"  ... {len(size_mismatches) - 50} more")
            issues += len(size_mismatches)

    print()
    if issues:
        print(f"Validation finished with {issues} issue(s).")
        return 1

    print("Validation finished without blocking issues.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate mask files against an image folder.")
    parser.add_argument("images", type=Path, help="Path to the image folder")
    parser.add_argument("masks", type=Path, help="Path to the mask folder")
    args = parser.parse_args()

    if not args.images.is_dir():
        parser.error(f"image folder not found: {args.images}")
    if not args.masks.is_dir():
        parser.error(f"mask folder not found: {args.masks}")

    return validate(args.images, args.masks)


if __name__ == "__main__":
    raise SystemExit(main())
