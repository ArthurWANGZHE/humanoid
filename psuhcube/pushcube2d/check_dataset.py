"""Validate and summarize a PushCube2D dataset."""

from __future__ import annotations

import argparse
from pathlib import Path

from .dataset import load_dataset, validate_dataset


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    args = parser.parse_args()

    data = load_dataset(args.dataset)
    issues, summary = validate_dataset(data)
    if issues:
        print("Dataset check failed:")
        for issue in issues:
            print(f"- {issue}")
        raise SystemExit(1)

    print("Dataset check passed.")
    for key, value in summary.items():
        if isinstance(value, float):
            print(f"{key}: {value:.4f}")
        else:
            print(f"{key}: {value}")


if __name__ == "__main__":
    main()

