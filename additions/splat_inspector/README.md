# Splat Inspector

Standalone PLY inspection utilities for Brush-related splat files.

This folder is not compiled by Brush. Run the script manually when you want to inspect a `.ply` file.

## Usage

```powershell
python additions\splat_inspector\inspect_ply.py samples\example_scene.ply
```

## What it reports

- PLY format: ASCII, binary little-endian, or binary big-endian
- Declared element counts
- Vertex property names and types
- Common Gaussian splat property coverage
- File size

The script reads only the PLY header by default, so it is fast on large files.
