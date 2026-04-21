# Dataset Tools

Standalone dataset validation utilities for Brush input folders.

This folder is not compiled by Brush. Run the script manually before training or testing a dataset.

## Usage

```powershell
python additions\dataset_tools\validate_dataset.py path\to\dataset
```

## What it checks

- The dataset path exists.
- Common image extensions are present.
- Image filenames are unique ignoring case.
- Known metadata files such as `transforms.json` or COLMAP sparse files are present.
- Empty folders and suspicious missing structure are reported.
