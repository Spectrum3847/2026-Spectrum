---
name: limelight-rewind
description: "Use after a match (or any time) to pull everything off the robot in one command: the roboRIO .wpilog files and the Limelight Rewind recordings from limelight-left, limelight-right and limelight-turret. It says SAFE TO DISCONNECT once every file is verified, then finds the match in each rewind, trims the video to it, and packages .rwnd files ready to upload to the Limelight Rewind Viewer (tools.limelightvision.io/rewind-viewer). Also for 'pull the rewinds', 'get the Limelight video', 'trim the rewind', 'package for the log analyzer'."
metadata:
  short-description: Pull RIO logs + Limelight rewinds, say when to unplug, trim and package .rwnd
---

# Limelight Rewind

A robot that just came off the field has two kinds of match data: the roboRIO's `.wpilog`, and a
Rewind recording on each Limelight 4 (`Robot.teleopExit()` saves the last 165 s on all cameras
when the FMS is attached). This skill pulls both, tells the pit crew the moment the robot can be
unplugged, then works offline to cut each video to the match and package it for the
[Limelight Rewind Viewer](https://tools.limelightvision.io/rewind-viewer).

## Core Rules

- **Run the script; do not curl the Limelights by hand.** One command does the pull, the checks,
  and the packaging.
- **Relay the SAFE / NOT SAFE line to the user first, word for word.** It is the thing they are
  waiting on. "NOT SAFE" lists exactly what did not come off; a device that has nothing to pull
  (no rewind saved) is not a reason to keep the robot tethered, so say which case it is.
- **Read-only on the robot.** Never call the Limelight's `/delete-video(s)` or `/recording-flush`
  routes, and never delete anything on the roboRIO. Old recordings age out of the Limelight's
  disk pool on their own.
- **Do not upload anything.** The viewer is a third-party site; give the user the folder and
  let them drag the `.rwnd` files in.
- **Report UNCONFIRMED matches.** If the script could not confirm which log a rewind belongs
  to, say so and show the evidence line; offer `--wpilog <file>` to pin it.

## Commands

Everything, after a match (robot on, laptop tethered or on the robot network):

```sh
python .agents/skills/limelight-rewind/scripts/pull_match_data.py
```

Only package recordings already on the laptop (offline):

```sh
python .agents/skills/limelight-rewind/scripts/package_rewind.py
```

Useful options:

- `pull_match_data.py --no-package`: pull and verify only.
- `pull_match_data.py --rio-count 5`: newest 5 `.wpilog` files instead of 3.
- `pull_match_data.py --limelight limelight-left=10.38.47.11`: a camera by IP when mDNS fails (repeatable; listing any replaces the default three).
- `package_rewind.py --pre 5 --post 3`: seconds kept around the match (default 3 and 3).
- `package_rewind.py --wpilog <file>`: skip the log search and use this log.
- `package_rewind.py --only limelight-turret --redo`: rebuild one camera's `.rwnd`.

Python 3.9+ standard library only; `ssh`/`scp` come with Windows 10+ and Git Bash. No ffmpeg
needed: the AVI is cut frame-exact in Python.

## Where things go (all gitignored)

| Path | What |
|---|---|
| `rio-logs/*.wpilog` | roboRIO logs (shared with `fast-log-triage`'s `pull_rio_logs.sh` and `tools/copy-match-logs.py`) |
| `logs/rewind/raw/<limelight>/rewind_*` | raw recordings: `.avi`, `_manifest.jsonl`, `_bootlog.txt.gz` |
| `logs/rewind/upload/<EVENT>_<MATCH>/` | `<EVENT>_<MATCH>_<limelight>.rwnd` per camera + `README.txt` (which log matched, what was cut) |

## How the match is found

- Every manifest line has `ts_nt`, the frame's NetworkTables time in microseconds. That is the
  roboRIO clock every `.wpilog` record is stamped with, so the match window comes straight from
  `DS:enabled` / `DS:autonomous` in the log. Checked on TXMAN1 Q20 (2026-10-16): auto 210.8 s to
  teleop end 374.3 s in the log; the rewinds ran 209.25 s to 374.24 s.
- NT time restarts on every RIO boot, so several logs cover the same range (Q17 overlaps Q20).
  The log is confirmed by comparing the Limelight's `botpose_wpiblue` / `botpose_orb_wpiblue` with
  the robot's `Vision/<camera>/MT1Pose` / `MT2Pose` at the same instant. The right log agrees
  within a few cm (Q20: 1.5 to 3.4 cm); a wrong one is metres off (Q17: 6 to 7 m). Below 0.30 m
  median over 20+ frames counts as confirmed.
- The Limelight wall clock (`ts_sys`, and the `rewind_YYYYMMDD_HHMMSS` name) is wrong on our
  cameras (2024 and 2025 dates), so neither is used.

## The .rwnd format

A zip, stored without compression, holding `<name>.avi` (MJPEG), `<name>_manifest.jsonl` (a
header line, then one JSON line per video frame), and `<name>_bootlog.txt.gz`. The viewer finds
the members by suffix, pairs AVI frame *i* with manifest line *i*, and syncs its slots on `ts_nt`.
The trimmed manifest keeps the original header plus a `spectrum3847` block naming the source
recording, the wpilog, the evidence, and the source frame range.

## What it cannot do

- A recording only exists if something saved one. The robot saves on `teleopExit` only with the
  FMS attached. For a practice match, press the "Record last 2m 50s" Rewind button in the
  Limelight web UI as soon as the match ends: it keeps only the last 170 s, and a match is
  ~163.5 s, so a few seconds' delay already costs the start of auto.
- **Playoffs at TXMAN1 ran as FMS test matches** (2026-10-17): FMS attached, event `TXMAN1`,
  match number 999, match type None. WPILib does not rename those logs, so the upload folder is
  named by the log's timestamp (`FRC_20261017_183511`). The rewind still saves (FMS is attached).
  Pass `package_rewind.py --name TXMAN1_E5 --only <recording> --redo` to name it for the match.
- **Pull after every match.** How many recordings a Limelight keeps varies (its disk pool drops
  the oldest when it fills). On 2026-10-16 every `/videolist` held exactly one and Q26's rewind
  was gone once Q28 saved; on 2026-10-17 limelight-left still held both Q32 and Q34 while right and
  turret held only a later one. The roboRIO keeps its logs, so a missed match still has its
  `.wpilog`, just no video.
- The robot saves exactly 165 s at teleop end and a match is ~163.5 s, so a match-day rewind
  loses only the ~1.5 s before auto. Trimming matters for longer manual saves and for cutting
  pre-match idle time.
- The Limelight routes `/videolist` and `/recording/<file>` (port 5807) were read from Limelight
  OS's own web UI on 2026-10-16; the full pull ran end to end against the robot that day
  (TXMAN1 Q22: 3 RIO logs + 3 rewinds, about 130 s over the tether). The wait on `/results`
  `rewind.flushing` before listing has not been exercised against a camera mid-save yet. A future
  Limelight OS can move these; if `/videolist` 404s, look at the Rewind tab's network calls.
