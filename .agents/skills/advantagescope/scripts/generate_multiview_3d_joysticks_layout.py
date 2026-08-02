#!/usr/bin/env python3
"""Generate an AdvantageScope 3D Field hub with a Joysticks satellite."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def expanded_paths(topics: list[str]) -> list[str]:
    expanded: list[str] = []
    seen: set[str] = set()
    for topic in topics:
        parts = [part for part in topic.split("/") if part]
        current = ""
        for part in parts[:-1]:
            current += "/" + part
            if current not in seen:
                expanded.append(current)
                seen.add(current)
    return expanded


def joystick_layouts(port: int, layout: str, max_ports: int = 6) -> list[str]:
    layouts = ["None"] * max_ports
    if port < 0 or port >= max_ports:
        raise SystemExit(f"Joystick port must be in [0, {max_ports - 1}], got {port}")
    layouts[port] = layout
    return layouts


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True, help="Layout JSON path")
    parser.add_argument("--field", required=True, help="AdvantageScope 3D field id")
    parser.add_argument("--robot", required=True, help="AdvantageScope robot model name")
    parser.add_argument("--pose-topic", default="/RealOutputs/Swerve/Odometry/Robot", help="Pose2d/Pose3d topic")
    parser.add_argument("--topic-type", default="Pose2d", choices=["Pose2d", "Pose3d"], help="Robot source log type")
    parser.add_argument("--game-piece-topic", help="Pose3d[] topic for field game pieces")
    parser.add_argument("--game-piece-variant", default="Fuel", help="AdvantageScope 3D game-piece variant")
    parser.add_argument("--trajectory-topic", help="Pose3d[] topic for a 3D trajectory line")
    parser.add_argument("--trajectory-color", default="orange", help="AdvantageScope trajectory color")
    parser.add_argument("--trajectory-size", default="bold", choices=["normal", "bold"], help="Trajectory line thickness")
    parser.add_argument("--joystick-port", type=int, default=0, help="Joystick port to display")
    parser.add_argument("--joystick-layout", default="Xbox Controller", help="AdvantageScope joystick layout name")
    parser.add_argument("--selected-tab", choices=["field", "joysticks"], default="field")
    args = parser.parse_args()

    field_sources = [
        {
            "type": "robot",
            "logKey": args.pose_topic,
            "logType": args.topic_type,
            "visible": True,
            "options": {
                "model": args.robot,
            },
        }
    ]
    if args.game_piece_topic:
        field_sources.append(
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
        field_sources.append(
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

    field_controller = {
        "sources": field_sources,
        "game": args.field,
    }
    joystick_controller = joystick_layouts(args.joystick_port, args.joystick_layout)

    state = {
        "sidebar": {
            "width": 260,
            "expanded": expanded_paths([
                args.pose_topic,
                args.game_piece_topic or "",
                args.trajectory_topic or "",
                f"/DriverStation/Joystick{args.joystick_port}/AxisValues",
                f"/DriverStation/Joystick{args.joystick_port}/ButtonValues",
                f"/DriverStation/Joystick{args.joystick_port}/POVs",
            ]),
        },
        "tabs": {
            "selected": 0 if args.selected_tab == "field" else 1,
            "tabs": [
                {
                    "type": 3,
                    "title": "3D Field",
                    "controller": field_controller,
                    "controllerUUID": "agent-field3d",
                    "renderer": None,
                    "controlsHeight": 200,
                },
                {
                    "type": 8,
                    "title": "Joysticks",
                    "controller": joystick_controller,
                    "controllerUUID": "agent-joysticks",
                    "renderer": None,
                    "controlsHeight": 200,
                },
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
        "satellites": [
            {
                "x": 1420,
                "y": 39,
                "width": 820,
                "height": 720,
                "uuid": "agent-joysticks",
                "state": {
                    "type": 8,
                    "visualizer": None,
                },
            }
        ],
    }

    out = Path(args.out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(layout, indent=2) + "\n")
    print(out)


if __name__ == "__main__":
    main()
