#!/usr/bin/env python3
"""Generate a minimal AdvantageScope Line Graph layout."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ascope_layout_common import expanded_paths, next_color, parse_range, source


def main() -> None:
    """Generate a minimal AdvantageScope line graph layout JSON file."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True, help="Layout JSON path")
    parser.add_argument("--left-topic", action="append", required=True, help="Numeric topic for the left axis")
    parser.add_argument("--right-topic", action="append", default=[], help="Numeric topic for the right axis")
    parser.add_argument("--discrete-topic", action="append", default=[], help="Boolean/discrete topic")
    parser.add_argument("--title", default="Line Graph", help="Tab title")
    parser.add_argument("--left-range", nargs=2, type=float, metavar=("MIN", "MAX"), help="Locked left axis range")
    parser.add_argument("--right-range", nargs=2, type=float, metavar=("MIN", "MAX"), help="Locked right axis range")
    args = parser.parse_args()

    left_sources = [source(topic, next_color(), "smooth") for topic in args.left_topic]
    right_sources = [source(topic, next_color(), "smooth") for topic in args.right_topic]
    discrete_sources = [source(topic, next_color(), "graph") for topic in args.discrete_topic]
    all_topics = args.left_topic + args.right_topic + args.discrete_topic

    state = {
        "sidebar": {
            "width": 260,
            "expanded": expanded_paths(all_topics),
        },
        "tabs": {
            "selected": 0,
            "tabs": [
                {
                    "type": 1,
                    "title": args.title,
                    "controller": {
                        "leftSources": left_sources,
                        "rightSources": right_sources,
                        "discreteSources": discrete_sources,
                        "leftLockedRange": parse_range(args.left_range),
                        "rightLockedRange": parse_range(args.right_range),
                        "leftUnitConversion": {"autoTarget": None, "preset": None},
                        "rightUnitConversion": {"autoTarget": None, "preset": None},
                        "leftFilter": 0,
                        "rightFilter": 0,
                    },
                    "controllerUUID": "agent-linegraph",
                    "renderer": None,
                    "controlsHeight": 300,
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
