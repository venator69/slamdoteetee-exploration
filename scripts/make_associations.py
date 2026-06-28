#!/usr/bin/env python3
"""Create TUM RGB-D associations.txt from rgb.txt and depth.txt."""

from __future__ import annotations

import argparse
from pathlib import Path


def read_file_list(filename: Path, remove_bounds: bool = False) -> dict[float, list[str]]:
    lines = filename.read_text().replace(",", " ").replace("\t", " ").splitlines()
    if remove_bounds:
        lines = lines[100:-100]
    entries: dict[float, list[str]] = {}
    for line in lines:
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        parts = [part for part in line.split() if part]
        if len(parts) < 2:
            continue
        entries[float(parts[0])] = parts[1:]
    return entries


def associate(
    first_list: dict[float, list[str]],
    second_list: dict[float, list[str]],
    offset: float,
    max_difference: float,
) -> list[tuple[float, float]]:
    first_keys = set(first_list)
    second_keys = set(second_list)
    potential_matches = [
        (abs(a - (b + offset)), a, b)
        for a in first_keys
        for b in second_keys
        if abs(a - (b + offset)) < max_difference
    ]
    potential_matches.sort()
    matches: list[tuple[float, float]] = []
    for _, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))
    matches.sort()
    return matches


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("seq_dir", type=Path)
    parser.add_argument("--offset", type=float, default=0.0)
    parser.add_argument("--max-difference", type=float, default=0.02)
    args = parser.parse_args()

    rgb_file = args.seq_dir / "rgb.txt"
    depth_file = args.seq_dir / "depth.txt"
    out_file = args.seq_dir / "associations.txt"

    rgb = read_file_list(rgb_file)
    depth = read_file_list(depth_file)
    matches = associate(rgb, depth, args.offset, args.max_difference)

    lines = []
    for rgb_ts, depth_ts in matches:
        lines.append(
            f"{rgb_ts} {' '.join(rgb[rgb_ts])} {depth_ts - args.offset} {' '.join(depth[depth_ts])}"
        )
    out_file.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote {len(lines)} associations to {out_file}")


if __name__ == "__main__":
    main()
