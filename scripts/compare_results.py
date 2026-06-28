#!/usr/bin/env python3
"""Generate comparison table across fusion modes."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from kitti_gps_ekf.pipeline import compare_all_methods, load_config


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--project-root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()

    project = args.project_root
    config = load_config(project / "config" / "fusion_config.yaml")
    results = project / "results" / args.sequence
    df = compare_all_methods(config, args.sequence, results)
    print(results / "comparison_table.csv")
    print(results / "comparison_table.md")
    print(df.to_string(index=False))


if __name__ == "__main__":
    main()
