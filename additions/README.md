# Additions

This directory contains standalone helper tools for datasets, masks, and splat files.

The code in this directory is intentionally not part of the main Brush build:

- It is outside `crates/`.
- It is excluded from the Cargo workspace in the root `Cargo.toml`.
- It is not imported by Rust, TypeScript, or build scripts.
- Each tool is run manually from the command line.

Do not add this directory to `workspace.members`, `package.json` scripts, or any `build.rs` file unless the tool is intentionally being promoted into the main project.

## Tools

- `splat_inspector/inspect_ply.py`: Inspect ASCII or binary PLY headers and report vertex/splat metadata.
- `dataset_tools/validate_dataset.py`: Check common dataset layout issues before training.
- `mask_tools/validate_masks.py`: Check mask coverage and image-size compatibility.

## Manual usage

Run a tool directly with Python:

```powershell
python additions\splat_inspector\inspect_ply.py samples\example_scene.ply
python additions\dataset_tools\validate_dataset.py path\to\dataset
python additions\mask_tools\validate_masks.py path\to\images path\to\masks
```
