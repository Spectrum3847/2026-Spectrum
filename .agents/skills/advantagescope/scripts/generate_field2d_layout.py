#!/usr/bin/env python3
"""Generate a minimal AdvantageScope 2D field layout."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> None:
    """Generate a minimal AdvantageScope 2D field layout JSON file."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True, help="Layout JSON path")
    parser.add_argument("--topic", default="/RealOutputs/Swerve/Odometry/Robot", help="Pose2d topic")
    parser.add_argument("--field", required=True, help="AdvantageScope field id")
    parser.add_argument("--title", default="2D Field", help="Tab title")
    args = parser.parse_args()

    state = {
        "sidebar": {
            "width": 260,
            "expanded": ["/RealOutputs", "/RealOutputs/Swerve", "/RealOutputs/Swerve/Odometry"],
        },
        "tabs": {
            "selected": 0,
            "tabs": [
                {
                    "type": 2,
                    "title": args.title,
                    "controller": {
                        "sources": [
                            {
                                "type": "robot",
                                "logKey": args.topic,
                                "logType": "Pose2d",
                                "visible": True,
                                "options": {"bumpers": ""},
                            }
                        ],
                        "field": args.field,
                        "orientation": 0,
                        "size": "large",
                    },
                    "controllerUUID": "agent-field2d",
                    "renderer": None,
                    "controlsHeight": 200,
                }
            ],
        },
    }
    layout = {
        "version": "26.0.2",
        "hubs": [
            {
                "x": 0,
                "y": 39,
                "width": 1400,
                "height": 900,
                "state": state,
            }
        ],
        "satellites": [],
    }

    out = Path(args.out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(layout, indent=2) + "\n")
    print(out)


if __name__ == "__main__":
    main()
