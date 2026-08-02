#!/usr/bin/env python3
"""Generate an AdvantageScope hub + satellite multi-view layout."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


COLORS = [
    "#ff0000",
    "#00a000",
    "#0066ff",
    "#ff9900",
    "#aa00ff",
    "#00aaaa",
    "#ff66cc",
    "#999999",
]


def source(topic: str, color: str, source_type: str) -> dict:
    return {
        "type": source_type,
        "logKey": topic,
        "logType": "Number" if source_type == "smooth" else "Boolean",
        "visible": True,
        "options": {"color": color, "size": "bold"} if source_type == "smooth" else {"color": color},
    }


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


def parse_range(raw: list[float] | None) -> list[float] | None:
    return None if raw is None else [raw[0], raw[1]]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True, help="Layout JSON path")
    parser.add_argument("--field", required=True, help="AdvantageScope 3D field id")
    parser.add_argument("--robot", required=True, help="AdvantageScope robot model name")
    parser.add_argument("--pose-topic", default="/RealOutputs/Swerve/Odometry/Robot", help="Pose2d/Pose3d topic")
    parser.add_argument("--topic-type", default="Pose2d", choices=["Pose2d", "Pose3d"], help="Robot source log type")
    parser.add_argument("--left-topic", action="append", required=True, help="Numeric topic for the graph left axis")
    parser.add_argument("--right-topic", action="append", default=[], help="Numeric topic for the graph right axis")
    parser.add_argument("--discrete-topic", action="append", default=[], help="Boolean/discrete graph topic")
    parser.add_argument("--title", default="Line Graph", help="Graph tab title")
    parser.add_argument("--left-range", nargs=2, type=float, metavar=("MIN", "MAX"), help="Locked left axis range")
    parser.add_argument("--right-range", nargs=2, type=float, metavar=("MIN", "MAX"), help="Locked right axis range")
    args = parser.parse_args()

    color_index = 0

    def next_color() -> str:
        nonlocal color_index
        color = COLORS[color_index % len(COLORS)]
        color_index += 1
        return color

    left_sources = [source(topic, next_color(), "smooth") for topic in args.left_topic]
    right_sources = [source(topic, next_color(), "smooth") for topic in args.right_topic]
    discrete_sources = [source(topic, next_color(), "graph") for topic in args.discrete_topic]
    graph_topics = args.left_topic + args.right_topic + args.discrete_topic

    field_controller = {
        "sources": [
            {
                "type": "robot",
                "logKey": args.pose_topic,
                "logType": args.topic_type,
                "visible": True,
                "options": {
                    "model": args.robot,
                },
            }
        ],
        "game": args.field,
    }
    graph_controller = {
        "leftSources": left_sources,
        "rightSources": right_sources,
        "discreteSources": discrete_sources,
        "leftLockedRange": parse_range(args.left_range),
        "rightLockedRange": parse_range(args.right_range),
        "leftUnitConversion": {"autoTarget": None, "preset": None},
        "rightUnitConversion": {"autoTarget": None, "preset": None},
        "leftFilter": 0,
        "rightFilter": 0,
    }

    state = {
        "sidebar": {
            "width": 260,
            "expanded": expanded_paths([args.pose_topic, *graph_topics]),
        },
        "tabs": {
            "selected": 0,
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
                    "type": 1,
                    "title": args.title,
                    "controller": graph_controller,
                    "controllerUUID": "agent-linegraph",
                    "renderer": None,
                    "controlsHeight": 300,
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
                "width": 1200,
                "height": 720,
                "uuid": "agent-linegraph",
                "state": {
                    "type": 1,
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
