---
name: advantagescope
description: "Use for AdvantageScope visualization automation: building/running our AdvantageScope fork (Spectrum 3847) or vanilla AdvantageScope, opening WPILOGs/DataLogs with layouts, using opt-in agent control/export hooks, capturing 2D/3D field screenshots or frame sequences, and falling back to vanilla AdvantageScope/manual inspection when hooks are unavailable."
metadata:
  short-description: Automate AdvantageScope visualization
---

# AdvantageScope

> We maintain our own AdvantageScope fork; these workflow facts were verified against the Spectrum fork and robot repo 2026-08-07 (PR #132).

## Core Rules

- This skill is the robot-repo consumer workflow for AdvantageScope visualization. It should help agents discover robot logs/assets, generate layouts, locate our AdvantageScope fork or vanilla AdvantageScope, and export previews.
- Keep fork maintenance details in the AdvantageScope repo's `$advantagescope-fork` skill (we maintain our own fork). Use that skill when modifying, rebasing, packaging, or debugging fork internals.
- Do not hard-code season-specific field ids, robot model names, autos, or layouts. Discover custom assets from `AScope_Assets` and use explicit task arguments.
- Do not automate by screen clicks or fixed coordinates. Use local agent control/export hooks when available.
- If only vanilla AdvantageScope is available, open the log/layout manually and use WPILOG parsing for objective results.
- When a new repeatable AdvantageScope workflow is discovered, treat updating this skill as part of the work. Add the durable instructions and, when the workflow is more than a couple commands, add or improve a generic helper script so future agents can use the app immediately without rediscovering the path.

## Build And Runtime

- Prefer our AdvantageScope fork for agent exports. If the fork is not checked out locally, ask the user before cloning/building it because that requires network, disk, and dependency changes; otherwise fall back to vanilla AdvantageScope plus WPILOG summaries.
- `export_preview.py` can also use `ADVANTAGESCOPE_EXECUTABLE` or `--advantagescope <executable>` when a packaged/built fork executable is already available.
- Do not copy robot assets into the fork. Custom robot assets should stay in this robot repo's `AScope_Assets` folder and be loaded through AdvantageScope's user assets feature.

## Agent Hooks

- Before exporting an auto preview, get the WPILOG from `$wpilib-sim`'s permanent workflow:

  ```sh
  python3 .agents/skills/wpilib-sim/scripts/run_auto_sim.py \
    --repo . \
    --auto <auto-name> \
    --alliance <alliance-station> \
    --duration <auto-duration> \
    --buffer 1
  ```

  Then compute the auto window from logged DriverStation state:

  ```sh
  python3 .agents/skills/wpilib-sim/scripts/find_wpilog_window.py \
    --repo . \
    --log <log.wpilog> \
    --all-true /DriverStation/Autonomous \
    --all-true /DriverStation/Enabled \
    --duration <auto-duration>
  ```
- Preferred export command from a built fork:

  ```sh
  ./node_modules/.bin/electron bundles/main.js \
    --agent-export \
    --log <log.wpilog> \
    --layout <layout.json> \
    --out <output-dir> \
    --start <seconds> \
    --end <seconds> \
    --fps 30 \
    --agent-assets <assets-folder> \
    --agent-export-mode realtime \
    --agent-headless
  ```
- For auto previews, use a real-time export window anchored to the logged auto start instead of the start of the file. Prefer the first timestamp where `/DriverStation/Autonomous` and `/DriverStation/Enabled` are both true, then export `--start <auto-start> --end <auto-start + auto-duration> --fps 30`.
- Realtime export records the active 2D/3D field canvas while AdvantageScope playback runs at 1x. A current fork waits for selected-renderer readiness before recording, so 3D field assets should be loaded in frame 0 without a fixed delay. A preview should take roughly the requested video duration plus small startup/transcode overhead.
- For agent exports, prefer `--agent-headless` instead of Chromium `--headless`. It keeps AdvantageScope windows hidden while preserving the normal renderer/WebGL lifecycle needed for 3D recording.
- The skill includes helper scripts:

  ```sh
  python3 .agents/skills/advantagescope/scripts/discover_ascope_assets.py \
    --repo .

  python3 .agents/skills/advantagescope/scripts/generate_field2d_layout.py \
    --out artifacts/auto-previews/layouts/ascope-2d-layout.json \
    --topic /RealOutputs/Swerve/Odometry/Robot \
    --field "<field-id>"

  python3 .agents/skills/advantagescope/scripts/generate_field3d_layout.py \
    --out artifacts/auto-previews/layouts/ascope-3d-layout.json \
    --topic /RealOutputs/Swerve/Odometry/Robot \
    --field "<field-id>" \
    --robot "<robot-model>"

  python3 .agents/skills/advantagescope/scripts/export_preview.py \
    --log <log.wpilog> \
    --layout artifacts/auto-previews/layouts/ascope-2d-layout.json \
    --out artifacts/auto-previews/<name> \
    --start <auto-start> \
    --end <auto-start + auto-duration> \
    --assets AScope_Assets \
    --headless \
    --fps 30
  ```
- Agent realtime export should produce `preview.webm` directly from the renderer. It should also produce `preview.mp4` when bundled FFmpeg conversion succeeds.
- Use `--agent-export-mode multiview` for synced hub + satellite realtime videos. The fork records the selected hub canvas and the first satellite canvas at the same time, then composes a side-by-side `preview.mp4` with bundled FFmpeg.
- Use `--agent-export-mode frames` only for debugging deterministic frame stepping; normal auto previews must not use the frame-by-frame path.
- Use `--agent-allow-frame-only` only with frame export debugging when no encoder is available.
- Optional control mode:

  ```sh
  ./node_modules/.bin/electron bundles/main.js --agent-control --agent-port 0
  ```
- The control API must bind only to `127.0.0.1`, require token auth, and keep commands small: `status`, `openLog`, `applyLayout`, `setTime`, `capture`, `export`, and `close`.
- Export readiness must come from AdvantageScope state, not a blind sleep. A reliable status includes loaded log path, timestamp range, selected time, active tab, source status, missing sources, missing assets, and latest main/renderer error.
- Fail clearly before capture when:
  - the log path is missing or cannot load,
  - the layout path is missing or invalid,
  - the active layout references missing log topics,
  - the requested 2D/3D field asset is unavailable.

## Line Graph Captures

- Use Line Graph tab type `1` for static plotted data from WPILOGs.
- A Line Graph controller state uses `leftSources`, `rightSources`, and `discreteSources`. Numeric source entries should use `type: "smooth"`, `logType: "Number"`, `visible: true`, and `options` with `color` and `size`. Discrete boolean overlays can use `type: "graph"` with `logType: "Boolean"`.
- Prefer `capture_layout.py` for graph screenshots. It launches the fork in `--agent-control --agent-headless`, waits for `liveStatus.ready=true`, optionally sends `setTime`, sends `capture`, and closes cleanly.
- Pair graph screenshots with `$wpilib-sim` topic summaries from `read_wpilog_values.py` so users get both the visual trend and concrete min/max values.
- Fast path for current-style graph requests:

  ```sh
  python3 .agents/skills/wpilib-sim/scripts/list_wpilog_topics.py \
    --repo . \
    --log <log.wpilog> \
    --filter Current

  python3 .agents/skills/advantagescope/scripts/generate_linegraph_layout.py \
    --out artifacts/auto-previews/layouts/drive-current-linegraph.json \
    --title "Drive Supply Current" \
    --left-topic /Swerve/Module0/DriveSupplyCurrentAmps \
    --left-topic /Swerve/Module1/DriveSupplyCurrentAmps \
    --left-topic /Swerve/Module2/DriveSupplyCurrentAmps \
    --left-topic /Swerve/Module3/DriveSupplyCurrentAmps \
    --discrete-topic /DriverStation/Autonomous \
    --discrete-topic /DriverStation/Enabled \
    --left-range 0 80

  python3 .agents/skills/advantagescope/scripts/capture_layout.py \
    --log <log.wpilog> \
    --layout artifacts/auto-previews/layouts/drive-current-linegraph.json \
    --out artifacts/auto-previews/drive-current-graph/drive-current.png \
    --time <auto-start + 10> \
    --headless
  ```

## Multi-View Previews

- AdvantageScope's native multi-view model is a hub window plus satellite windows. The hub owns the timeline; satellites subscribe to tab controllers and stay synchronized as the hub plays, scrubs, or changes selected time.
- Use `scripts/generate_multiview_layout.py` for a 3D Field + Line Graph preview. The helper creates:
  - hub tab 0: selected 3D Field tab,
  - hub tab 1: Line Graph controller tab,
  - satellite window: attached to the Line Graph controller UUID.
- Use `scripts/generate_multiview_3d_joysticks_layout.py` for 3D Field + Joysticks layouts. The helper can select the field tab or joystick tab with `--selected-tab field|joysticks` and can select a specific joystick layout for a port, for example `--joystick-port 0 --joystick-layout "Xbox Controller (White)"`.
- For game pieces, add `--game-piece-topic <Translation3d[] topic>` and `--game-piece-variant <variant>`. For fuel, use `/Robot/Sim/Fuel/Positions` (all pieces) or `/Robot/Sim/Fuel/InFlight` (airborne only) and variant `Fuel`.
- For projectile or path overlays, add `--trajectory-topic <Translation3d[] topic>`. For our fuel shots, use `/Robot/Sim/Fuel/LastShotArc`; the helper adds it as a 3D Field `trajectory` source with a bold orange line by default.
- For robot sources that include Z/pitch/roll, pass `--topic-type Pose3d` to `scripts/generate_field3d_layout.py`. For MapleSim bump traversal, use `Sim/RobotPose3d` (logged under `/Robot/` by DogLog) with `--topic-type Pose3d`.
- Use explicit field and robot model arguments from the task or asset discovery. Do not bake season-specific asset ids into the skill or helper defaults.
- Export with `export_preview.py --mode multiview --headless`. Expected outputs are `view-0-hub.webm`, `view-1-satellite.webm`, `preview.mp4`, and `manifest.json`.
- Joystick multiview is supported by the fork's renderer-owned canvas capture path. Validate manifests report the joystick view with `sourceMode: "renderer-canvas"` and `sourceName: "JoysticksRenderer"`.
- Fast path for a 3D teleop preview plus live Xbox joystick pane:

  ```sh
  python3 .agents/skills/advantagescope/scripts/generate_multiview_3d_joysticks_layout.py \
    --out artifacts/auto-previews/layouts/teleop-3d-joysticks.json \
    --field "<field-id>" \
    --robot "<robot-model>" \
    --pose-topic /Robot/Sim/RobotPose3d \
    --game-piece-topic /Robot/Sim/Fuel/Positions \
    --game-piece-variant Fuel \
    --trajectory-topic /Robot/Sim/Fuel/LastShotArc \
    --joystick-port 0 \
    --joystick-layout "Xbox Controller (White)" \
    --selected-tab field

  python3 .agents/skills/advantagescope/scripts/export_preview.py \
    --log <log.wpilog> \
    --layout artifacts/auto-previews/layouts/teleop-3d-joysticks.json \
    --out artifacts/auto-previews/teleop-3d-joysticks \
    --start <enabled-start> \
    --end <enabled-end> \
    --fps 30 \
    --assets AScope_Assets \
    --headless \
    --mode multiview
  ```
- Fast path for a 3D auto preview synced with a four-topic current graph:

  ```sh
  python3 .agents/skills/advantagescope/scripts/generate_multiview_layout.py \
    --out artifacts/auto-previews/layouts/multiview-3d-drive-current.json \
    --field "<field-id>" \
    --robot "<robot-model>" \
    --pose-topic /RealOutputs/Swerve/Odometry/Robot \
    --left-topic /Swerve/Module0/DriveSupplyCurrentAmps \
    --left-topic /Swerve/Module1/DriveSupplyCurrentAmps \
    --left-topic /Swerve/Module2/DriveSupplyCurrentAmps \
    --left-topic /Swerve/Module3/DriveSupplyCurrentAmps \
    --discrete-topic /DriverStation/Autonomous \
    --discrete-topic /DriverStation/Enabled \
    --left-range 0 80 \
    --title "Drive Supply Current"

  python3 .agents/skills/advantagescope/scripts/export_preview.py \
    --log <log.wpilog> \
    --layout artifacts/auto-previews/layouts/multiview-3d-drive-current.json \
    --out artifacts/auto-previews/multiview-3d-drive-current \
    --start <auto-start> \
    --end <auto-start + auto-duration> \
    --fps 30 \
    --assets AScope_Assets \
    --headless \
    --mode multiview
  ```
- Validate the multiview manifest has `mode: "multiview"`, a nonempty `views` array, `video` pointing to `preview.mp4`, no missing sources/assets, and field renderer readiness in the hub status. Inspect frame 0 and a mid-run frame to confirm the 3D field is loaded, the robot model is visible, and the satellite pane is visible. For Joysticks, confirm the satellite view reports `sourceName: "JoysticksRenderer"` and the controller overlay changes during playback.

## Layouts

- Generate layout JSON outside the fork, preferably under an ignored artifact directory in the robot repo or an OS-selected temp directory. Prefer `scripts/generate_field2d_layout.py` for 2D field previews and `scripts/generate_field3d_layout.py` for 3D field previews.
- For generic topic graphs, prefer `scripts/generate_linegraph_layout.py` and `scripts/capture_layout.py`. For synced visual + graph videos, prefer `scripts/generate_multiview_layout.py` and `scripts/export_preview.py --mode multiview`.
- Before generating layouts, run `scripts/discover_ascope_assets.py --repo .` to list custom assets in the robot repo's `AScope_Assets` folder. Use the reported robot model names for 3D layouts, and use reported custom field names or AdvantageScope built-in field ids for the current game.
- A minimal 2D field layout should select `TabType.Field2d` (`type: 2`), set the desired field id, and include one or more source entries with topic names supplied by the current task.
- A minimal 3D field layout should select `TabType.Field3d` (`type: 3`), set the desired field id, and add a `robot` source whose `options.model` names the requested AdvantageScope robot model.
- Do not hardcode season-specific field or model names in this skill. Field ids and robot models must come from the task, asset discovery, repo notes, or AdvantageScope's current built-in assets.
- Keep team-specific layout choices in the calling skill or task, not in the fork.

## Outputs

- Expect `preview.webm`, usually `preview.mp4`, plus `manifest.json` for normal realtime exports. Auto-review videos should be exported at real-time duration, beginning at the logged autonomous-enabled timestamp.
- If `--agent-export-mode frames --agent-allow-frame-only` is set, PNG frames plus `manifest.json` are acceptable debug output; do not present them as a completed video export.
- Always inspect at least one exported frame or screenshot for nonblank output, expected field asset, and visible robot/trajectory data.
- Pair visual artifacts with WPILOG-derived data summaries when the user is evaluating robot behavior.

## Validation Pattern

1. Run sim through `$wpilib-sim`, then derive the auto-enabled export window from the WPILOG.
2. Discover custom assets with `discover_ascope_assets.py`, generate a layout with explicit field/model arguments, and export with `export_preview.py`.
3. Check `manifest.json` for the requested mode, selected renderer readiness, no missing sources/assets, valid video output, and no main/renderer error. Multiview exports should include per-view webm files plus composed `preview.mp4`.
4. Confirm wall-clock export time is close to the requested video duration, then inspect frame 0 and a mid-run frame for nonblank output, expected field asset, and visible robot pose. If using an older fork without renderer readiness, use visual inspection fallback and only add a short explicit delay as a last resort.
5. If the fork code itself must change, move to the AdvantageScope checkout and use `$advantagescope-fork`.
