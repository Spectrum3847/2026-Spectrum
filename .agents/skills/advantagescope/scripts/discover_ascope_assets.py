#!/usr/bin/env python3
"""List custom AdvantageScope assets from a robot repo AScope_Assets folder."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def read_name(config_path: Path) -> str | None:
    """Read the "name" field from an AdvantageScope asset config.json."""
    try:
        data: Any = json.loads(config_path.read_text())
    except (OSError, json.JSONDecodeError):
        return None
    name = data.get("name")
    return name if isinstance(name, str) and name.strip() else None


def collect_assets(assets_dir: Path, prefix: str) -> list[dict[str, str]]:
    """Collect asset directories matching a prefix from an assets folder."""
    results: list[dict[str, str]] = []
    for child in sorted(assets_dir.glob(f"{prefix}_*")):
        if not child.is_dir():
            continue
        config_path = child / "config.json"
        name = read_name(config_path)
        if name is None:
            continue
        results.append({"name": name, "directory": str(child.resolve())})
    return results


def main() -> None:
    """List custom AdvantageScope assets from an AScope_Assets folder."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", default=".", help="Robot repo root")
    parser.add_argument("--assets", help="Custom AdvantageScope assets folder")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON")
    args = parser.parse_args()

    repo = Path(args.repo).resolve()
    assets_dir = Path(args.assets).resolve() if args.assets else repo / "AScope_Assets"
    if not assets_dir.is_dir():
        raise SystemExit(f"AdvantageScope assets folder not found: {assets_dir}")

    result = {
        "assetsFolder": str(assets_dir),
        "field2d": collect_assets(assets_dir, "Field2d"),
        "field3d": collect_assets(assets_dir, "Field3d"),
        "robots": collect_assets(assets_dir, "Robot"),
    }

    if args.json:
        print(json.dumps(result, indent=2))
        return

    print(f"assetsFolder={result['assetsFolder']}")
    for key, label in (("field2d", "2D fields"), ("field3d", "3D fields"), ("robots", "robots")):
        items = result[key]
        if items:
            print(f"{label}:")
            for item in items:
                print(f"  {item['name']}\t{item['directory']}")
        else:
            print(f"{label}: none in custom assets")
    if not result["field2d"] or not result["field3d"]:
        print("Use AdvantageScope built-in field ids for field assets not present in AScope_Assets.")


if __name__ == "__main__":
    main()
