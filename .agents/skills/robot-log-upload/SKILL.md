---
name: robot-log-upload
description: "Use when match logs need to come off a robot (roboRIO or SystemCore) and go up to the team log site (robotlog.spectrum3847.org): pulls .wpilog files over SSH, pulls Limelight rewind clips and packs them into .rwnd, uploads both, and confirms the match through the spectrum-logs MCP. Works for 3847 and sister teams (8515), on Windows, macOS and Linux."
metadata:
  short-description: Pull robot logs and Limelight rewinds, upload them to robotlog.spectrum3847.org
---

# Robot Log Upload

Robot → laptop → https://robotlog.spectrum3847.org, where the team scrubs matches and Claude reads
them through the `spectrum-logs` MCP. Scripts are in `scripts/`, Python 3.9+ standard library, and
use the OpenSSH `ssh`/`scp` that Windows 10+, macOS and Linux ship with. Each has `--help`.

## Rules

- **The upload key is secret and the repos are public.** Never ask for it in chat, type it, put it
  on a command line, or write it to a file in a repo. If `upload_logs.py --check` says it is not
  set, ask the user to run `python scripts/set_upload_key.py` in their own terminal (hidden prompt,
  saved in their home folder; a mentor has the key). The `LOG_SITE_KEY` environment variable also
  works and takes precedence.
- **The MCP is read-only**; uploads go through `upload_logs.py`, which runs the import script of the
  [WPI-Log-Analizer](https://github.com/Spectrum3847/WPI-Log-Analizer) repo (cloned next to this
  one, or at `LOG_ANALYZER_DIR`; Node 20+).
- **Uploading publishes** to a public site. Upload what was asked; ask before bulk-uploading shop or
  practice logs and before pulling rewinds (~360 MB per 165 s clip; R2's free tier is 10 GB).
- **Read-only on the robot and cameras**: never delete or change anything there.
- Pulls and rewind uploads take minutes; run them in the background.
- Re-uploading is safe: same name and size is skipped, and its analysis is refreshed.

## The robots

| Robot | Controller | SSH | Logs on the robot | Addresses |
|---|---|---|---|---|
| 3847 offseason bot (`2026-Spectrum`) | roboRIO, DogLog, `FRC_<date>_<time>[_<event>_<match>].wpilog` | `lvuser`, empty password, no prompt | `/U/logs`, `/home/lvuser/logs` | `10.38.47.2`, USB `172.22.11.2` |
| FM as 8515 (`2026-FM-SystemCore`) | SystemCore, AdvantageKit, `akit_<date>_<time>[_<event>_<match>].wpilog` | `systemcore`, this laptop's key (installed by its first `./gradlew deploy`) | `/U/logs`, `/home/systemcore/logs` | `10.85.15.2`, USB `172.26.0.1` |

Limelights serve rewind clips over HTTP on port 5807 (`/videolist`, `/recording/<file>`). 3847:
`limelight-left/right/turret.local`. 8515: `10.85.15.11` back, `.12` left, `.13` right. Robot code
saves a clip when the FMS is attached (165 s at teleop end; FM also 25 s at auto end). Camera clocks
reset every boot, so clip names are time since boot, not match time. The site wants one `.rwnd` per
clip: a stored zip of `<clip>.avi`, `<clip>_manifest.jsonl`, `<clip>_bootlog.txt.gz` (the web UI's
`:5801` Download button saves the same thing).

## Workflow

```sh
python scripts/upload_logs.py --check                                  # analyzer, node, key (never shown)
python scripts/pull_robot_logs.py --team 3847 -n 3 robot-logs/          # or --team 8515; --host/--user to force
python scripts/pull_limelight_rewinds.py list --team 8515               # see clips and sizes first
python scripts/pull_limelight_rewinds.py pull --team 8515 --latest 1 -o rewinds/
python scripts/pull_limelight_rewinds.py pack <folder of loose clip files> -o rewinds/
python scripts/upload_logs.py robot-logs/ rewinds/                      # logs go before rewinds automatically
```

`upload_logs.py` prints one result per file: `filed under match <id>`, `already on the site`,
`re-analyzed and moved to its match`, `uploaded, not matched to a log yet` (a rewind whose match log
is not up yet; upload it, or attach by hand on the site), or `FAILED: upload key required` (key
missing or wrong: ask the user to rerun `set_upload_key.py`). The team is read from the files;
`--team 8515` overrides it.

Then confirm with the MCP: `list_matches` (with `team`), `get_match` with the label (`"cc Q45"`,
`"8515 txman1 Q19"`). `files[].topicData` should be `true` and `rewinds` should list each camera.
Give the user the match `page` link.

## First-time setup on a laptop

```sh
git clone https://github.com/Spectrum3847/WPI-Log-Analizer     # next to this repo
python scripts/set_upload_key.py                                 # the user runs this
claude mcp add --scope user --transport http spectrum-logs https://robotlog.spectrum3847.org/mcp
```

`upload_logs.py` runs `npm install` in the analyzer on first use.

3847 only, when asked: real match logs under 50 MB are also committed to `2026-Spectrum/logs/matches/`
with `python tools/copy-match-logs.py` (see that repo's `AGENTS.md`).
