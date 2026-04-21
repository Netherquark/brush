# Mask Tools

Standalone mask validation utilities for image datasets.

This folder is not compiled by Brush. Run the script manually when checking masks before using them with a dataset.

## Usage

```powershell
python additions\mask_tools\validate_masks.py path\to\images path\to\masks
```

## What it checks

- Each image has a matching mask with the same stem.
- Each mask corresponds to an image.
- Image and mask dimensions match when Pillow is installed.

The dimension check is skipped if Pillow is not installed.
