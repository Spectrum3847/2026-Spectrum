#!/usr/bin/env python3
"""Generate a minimal AdvantageScope 3D field layout."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True, help="Layout JSON path")
    parser.add_argument("--topic", default="/RealOutputs/Swerve/Odometry/Robot", help="Pose2d/Pose3d topic")
    parser.add_argument("--topic-type", default="Pose2d", choices=["Pose2d", "Pose3d"], help="Robot source log type")
    parser.add_argument("--field", required=True, help="AdvantageScope 3D field id")
    parser.add_argument("--robot", required=True, help="AdvantageScope robot model name")
    parser.add_argument("--game-piece-topic", help="Pose3d[] topic for field game pieces")
    parser.add_argument("--game-piece-variant", default="Fuel", help="AdvantageScope 3D game-piece variant")
    parser.add_argument("--trajectory-topic", help="Pose3d[] topic for a 3D trajectory line")
    parser.add_argument("--trajectory-color", default="orange", help="AdvantageScope trajectory color")
    parser.add_argument("--trajectory-size", default="bold", choices=["normal", "bold"], help="Trajectory line thickness")
    parser.add_argument("--title", default="3D Field", help="Tab title")
    args = parser.parse_args()

    sources = [
        {
            "type": "robot",
            "logKey": args.topic,
            "logType": args.topic_type,
            "visible": True,
            "options": {
                "model": args.robot,
            },
        }
    ]
    if args.game_piece_topic:
        sources.append(
            {
                "type": "gamePiece",
                "logKey": args.game_piece_topic,
                "logType": "Pose3d[]",
                "visible": True,
                "options": {
                    "variant": args.game_piece_variant,
                },
            }
        )
    if args.trajectory_topic:
        sources.append(
            {
                "type": "trajectory",
                "logKey": args.trajectory_topic,
                "logType": "Pose3d[]",
                "visible": True,
                "options": {
                    "color": args.trajectory_color,
                    "size": args.trajectory_size,
                },
            }
        )
    state = {
        "sidebar": {
            "width": 260,
            "expanded": ["/RealOutputs", "/RealOutputs/Swerve", "/RealOutputs/Swerve/Odometry"],
        },
        "tabs": {
            "selected": 0,
            "tabs": [
                {
                    "type": 3,
                    "title": args.title,
                    "controller": {
                        "sources": sources,
                        "game": args.field,
                    },
                    "controllerUUID": "agent-field3d",
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
