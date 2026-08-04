#!/usr/bin/env python3
"""Shared helpers for AdvantageScope layout generators."""

from __future__ import annotations

from itertools import cycle

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
    """Build an AdvantageScope source dict for a numeric or boolean topic."""
    return {
        "type": source_type,
        "logKey": topic,
        "logType": "Number" if source_type == "smooth" else "Boolean",
        "visible": True,
        "options": {"color": color, "size": "bold"} if source_type == "smooth" else {"color": color},
    }


def expanded_paths(topics: list[str]) -> list[str]:
    """Expand hierarchical NetworkTables topic paths into parent prefixes."""
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
    """Copy a two-element range list, or return None when the input is None."""
    return None if raw is None else [raw[0], raw[1]]


def next_color() -> str:
    """Return the next color from the cycling color palette."""
    return next(_color_cycle)


_color_cycle = cycle(COLORS)


def build_field_sources(
    pose_topic: str,
    topic_type: str,
    robot: str,
    game_piece_topic: str | None = None,
    game_piece_variant: str = "Fuel",
    trajectory_topic: str | None = None,
    trajectory_color: str = "orange",
    trajectory_size: str = "bold",
) -> list[dict]:
    """Build AdvantageScope source entries for a 3D field layout."""
    sources = [
        {
            "type": "robot",
            "logKey": pose_topic,
            "logType": topic_type,
            "visible": True,
            "options": {
                "model": robot,
            },
        }
    ]
    if game_piece_topic:
        sources.append(
            {
                "type": "gamePiece",
                "logKey": game_piece_topic,
                "logType": "Pose3d[]",
                "visible": True,
                "options": {
                    "variant": game_piece_variant,
                },
            }
        )
    if trajectory_topic:
        sources.append(
            {
                "type": "trajectory",
                "logKey": trajectory_topic,
                "logType": "Pose3d[]",
                "visible": True,
                "options": {
                    "color": trajectory_color,
                    "size": trajectory_size,
                },
            }
        )
    return sources
