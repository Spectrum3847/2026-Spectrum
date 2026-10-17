---
name: robot-log-upload
description: Pull match logs (.wpilog) and Limelight rewind clips off a roboRIO or SystemCore and upload them to robotlog.spectrum3847.org. Use when match logs or rewinds need to go to the team log site, for 3847 or 8515.
---

# Robot log upload

This skill copies match files off the robot and uploads them to https://robotlog.spectrum3847.org.
The team reviews matches on that site, and agents read them through the read-only `spectrum-logs`
MCP. The scripts are in `scripts/`. They need Python 3.9+ and the OpenSSH client (`ssh`, `scp`)
that Windows 10+, macOS and Linux include. Each script has `--help`.

## Rules

- The upload key is a secret, and this repo is public. When `upload_logs.py --check` reports the
  key as not set, ask the user to run `python scripts/set_upload_key.py` in their own terminal. It
  reads the key at a hidden prompt and saves it in their home folder. A mentor has the key. The key
  stays out of chat, command lines and repo files. A `LOG_SITE_KEY` environment variable overrides
  the saved key.
- Uploads go through `upload_logs.py`. It runs `scripts/import.mjs` from
  [WPI-Log-Analizer](https://github.com/Spectrum3847/WPI-Log-Analizer), cloned next to this repo or
  at `LOG_ANALYZER_DIR`, with Node 20+.
- An upload is public. Upload only the files the user asked for. Ask before you bulk-upload shop or
  practice logs, and before you pull rewinds. One 165 s clip is about 360 MB, and the site has
  10 GB of storage.
- Only read from the robot and the cameras. Never delete or change files there.
- Pulls and rewind uploads take minutes. Run them in the background.
- Re-uploading is safe. The site skips a file with the same name and size, and re-analyzes a
  skipped log.

## Robots

| Robot | Controller and log name | SSH login | Log folders | Addresses |
|---|---|---|---|---|
| 3847 (`2026-Spectrum`) | roboRIO, DogLog, `FRC_<date>_<time>[_<event>_<match>].wpilog` | `lvuser`, empty password | `/U/logs`, `/home/lvuser/logs` | `10.38.47.2`, USB `172.22.11.2` |
| 8515 FM (`2026-FM-SystemCore`) | SystemCore, AdvantageKit, `akit_<date>_<time>[_<event>_<match>].wpilog` | `systemcore`, the laptop's SSH key from its first `./gradlew deploy` | `/U/logs`, `/home/systemcore/logs` | `10.85.15.2`, USB `172.26.0.1` |

Limelights serve rewind clips over HTTP on port 5807 (`/videolist`, `/recording/<file>`). The 3847
cameras are `limelight-left`, `-right`, `-turret` and `-back` on `.local`. The 8515 cameras are
`10.85.15.11` (back), `.12` (left) and `.13` (right).

Robot code saves a clip when the FMS is attached: 165 s at the end of teleop, and on FM also 25 s
at the end of auto. Camera clocks reset at every boot, so a clip name is the time since boot, not
the match time. The site takes one `.rwnd` per clip. That is a stored (uncompressed) zip of
`<clip>.avi`, `<clip>_manifest.jsonl` and `<clip>_bootlog.txt.gz`, the same file the Download
button in the camera web UI (port 5801) saves.

## Workflow

```sh
python scripts/upload_logs.py --check                                 # analyzer, node, key (never shown)
python scripts/pull_robot_logs.py --team 3847 -n 3 robot-logs/         # --host/--user to force one
python scripts/pull_limelight_rewinds.py list --team 8515              # clips and sizes
python scripts/pull_limelight_rewinds.py pull --team 8515 --latest 1 -o rewinds/
python scripts/pull_limelight_rewinds.py pack <folder of loose clip files> -o rewinds/
python scripts/upload_logs.py robot-logs/ rewinds/                     # uploads logs before rewinds
```

`upload_logs.py` prints one result per file:

- `filed under match <id>`
- `already on the site`, with `; re-analyzed and moved to its match` when a log moved
- `uploaded, not matched to a log yet`: the match log for this rewind is not on the site. Upload
  the log, or attach the rewind by hand on the site.
- `FAILED: ... 401 upload key required`: the key is missing or wrong. Ask the user to run
  `set_upload_key.py` again.

The script reads the team number from the files. `--team 8515` overrides it.

Confirm the upload with the MCP. Call `list_matches` with `team`, then `get_match` with the label
(`"cc Q45"`, `"8515 txman1 Q19"`). Each `files[].topicData` is `true`, and `rewinds` lists each
camera. Give the user the match `page` link.

## First-time setup on a laptop

```sh
git clone https://github.com/Spectrum3847/WPI-Log-Analizer    # next to this repo
python scripts/set_upload_key.py                                # the user runs this
claude mcp add --scope user --transport http spectrum-logs https://robotlog.spectrum3847.org/mcp
```

`upload_logs.py` runs `npm install` in the analyzer on the first upload.

For 3847, when the user asks, also commit real match logs under 50 MB to `logs/matches/` with
`python tools/copy-match-logs.py`. Both are on the `2026-offseason-bot` branch. See its `AGENTS.md`.
