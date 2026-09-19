# Match Logs

Real robot logs from real matches, committed so that every clone has them. The robot app's
`npm test` parses each file here, and the fast-log-triage script is checked against them
whenever a detector changes. Both were first written against synthetic logs, and both were
wrong about a real match log in ways a synthetic log could not show: the auto-movement
detector fired on a working auto because the pose snapped back to the start on the disable, and
the E-stop detector reported one stop as eight. Real data is the only fixture that catches that.

## Rules

- **Match-named logs only.** WPILib renames a log `FRC_YYYYMMDD_HHMMSS_<event>_<match>.wpilog` once
  the FMS attaches. Practice and shop logs never get the suffix, and they stay in the gitignored
  part of `logs/`. A log that is not match-named but belongs to a match (the restart half, say) can
  be forced in with `--include`.
- **Under 50 MB each.** GitHub blocks 100 MB and warns at 50, and every byte here is paid for by every
  clone forever. Bigger logs, and the whole season's worth, go to the
  [2026-Robot-Logs](https://github.com/Spectrum3847/2026-Robot-Logs) archive with `tools/archive-logs.sh`.
- **No `.hoot` files.** The Phoenix signal logs for one match run to 139 MB.
- **Describe every log below.** A log nobody can place is a log nobody will open.

## Adding one

```sh
python tools/copy-match-logs.py                                   # scan the usual download folders
python tools/copy-match-logs.py --include path/to/FRC_..._.wpilog  # a log that is not match-named
```

The script reads each copy back with the fast-log-triage reader before keeping it, so a truncated
download fails here rather than in a test later. Then add a row to the table and commit.

## What is here

|                        Log                        |                                                                      When and where                                                                       |                                                                                                                                                                                                                                               What happened                                                                                                                                                                                                                                                |                                                                                                    Good for testing                                                                                                     |
|---------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `FRC_20260801_223303_TXDRI1_Q20.wpilog` (19.8 MB) | TXDRI1, qualification 20, 2026-08-01. Log timestamps are UTC.                                                                                             | Auto was enabled for only 1.3 s (261.0 to 262.3 s); teleop ran 54 s (284.0 to 338.5 s) and ended in an E-stop. The Driver Station then toggled the E-stop flag eight times while disabled. No `BuildConstants` topics: this predates their logging.                                                                                                                                                                                                                                                        | E-stop handling and coalescing, a log without build metadata, the older topic set (261 topics).                                                                                                                         |
| `FRC_20260919_033032_cc_P8.wpilog` (10.1 MB)      | Chezy Champs, practice match 8, 2026-09-18 evening (file date is UTC). Robot `PM_2026`, code before d76c643.                                              | Both chassis cameras saw no tags for the whole 387 s pre-match, so the pose was never vision-seeded and the start-pose check read a vacuous 0.00 m. The auto drove, then the auto-to-teleop disable snapped the pose back to the auto start (407.6 s). At 446.3 s the CANivore bus was lost on a cut wire: every CANivore motor flapped connected/disconnected, the Pigeon disconnect alert fired at 446.9 s, and the rio-bus intake roller kept running. Battery sagged to 8.8 V at 250 to 290 A in auto. | Motor-disconnect detectors, the auto-movement detector (end minus start reads 0 here), vision seeding gates, CANivore versus rio bus health, battery dips. This is the match that produced commits d76c643 and 463c465. |
| `FRC_20260919_034444.wpilog` (0.5 MB)             | Same practice match, after the roboRIO was restarted from the Driver Station with the CANivore bus still dead. Never enabled. Forced in with `--include`. | Robot init took 64 s against 17 s the match before. Every CANivore motor reads disconnected from boot. CPU went from 40 to 50 percent to 95 to 100 percent at 99.7 s, exactly 30 s after init completed, with 58 to 77 percent of loops over 25 ms, and stayed there.                                                                                                                                                                                                                                      | Dead-bus boot, the CPU and loop-overrun detectors, a log with no enabled window at all.                                                                                                                                 |
