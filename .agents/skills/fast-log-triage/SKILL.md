---
name: fast-log-triage
description: "Use when a robot problem must be diagnosed FAST from one or more WPILOG files (between matches, in the pit, at practice): runs a one-command triage that ranks likely causes with timestamps and what to check on the robot, in under a minute, before any deep analysis."
metadata:
  short-description: Rank likely causes from a robot log in under a minute
---

# Fast Log Triage

The drive team has a few minutes between matches. This skill answers "what most likely
broke, and where do we look first" from the `.wpilog`, fast. It is a triage, not an
investigation: the deep dive lives in `$wpilog-decode` and `$advantagescope`, and only
happens after this has narrowed the search or when the user asks for it.

## Core Rules

- **Run the triage script first, before reading any robot source or decoding topics by hand.** One command, one pass over the file, done in seconds. Do not compile Java, do not launch AdvantageScope, do not open the code to "understand the subsystem" before the script has spoken.
- **Answer within the first reply.** Lead with the top one to three suspects, each with a timestamp and the physical thing to check on the robot. Then list what the script ruled out. Then stop. Offer the deep dive as one line; do not start it unasked.
- **Time-box any follow-up to the top suspect.** If the ranked list is not enough, decode only the topics named in that finding, only around its timestamp, with `read_wpilog_values.py` from `$wpilog-decode`. Do not survey the whole log.
- **Say what the log cannot tell you.** No `DS:enabled` topic means no match windows; no `BatteryLogger` topic means battery was not checked. The script prints these; pass them on rather than guessing.
- **Match the symptom to the finding.** If the user says "the turret was wrong" and the top finding is a launcher disconnect, say so, but lead with the turret findings (or their absence). The ranking is severity, not relevance.
- **Do not modify robot code from this skill.** The output is a diagnosis and a checklist for the pit crew. Code changes are a separate, user-approved step.
- No `~/wpilib`, Java, or network is needed. The reader is pure Python 3.9+ (`wpilog_fast.py`) and was checked record-for-record against WPILib's `DataLogReader`.

## Commands

Triage one log (a directory means its newest log):

```sh
python3 .agents/skills/fast-log-triage/scripts/triage_wpilog.py <match.wpilog | dir>
```

Options:

- `--all`: every finding including LOW (default shows CRITICAL/HIGH/MEDIUM, at most eight).
- `--tail 20`: append the last 20 console and `Prints` lines. Use when the log ends early or a finding says "check the console".
- `--latest 2 <dir>`: newest two logs from a directory. Two logs a minute apart is a robot code restart; the script says so.
- `--json`: machine-readable, for a script or a second agent.

Pull logs off the RIO first, if they are not local (read-only; tries `10.38.47.2`, then USB `172.22.11.2`, then mDNS; needs `ssh`, uses `sshpass` if installed so it does not prompt):

```sh
.agents/skills/fast-log-triage/scripts/pull_rio_logs.sh -n 2 ./rio-logs
python3 .agents/skills/fast-log-triage/scripts/triage_wpilog.py --latest 2 ./rio-logs
```

## What the Output Means

```
== match.wpilog  (48.2 MB, 214 s, 412 topics)
   build: main abc1234d 2026-09-18 19:02
   match: eventName=TXHOU matchType=2 matchNumber=42
   enabled: auto 30.0s→45.0s (15.0 s); teleop 48.0s→183.0s (135.0 s)

TOP SUSPECTS (ranked)
 1. [CRITICAL] Launcher motor DISCONNECTED for 7.4 s while enabled
      when:    80.0s (teleop+32.0s) → 87.4s
      evidence: Launcher/MotorConnected false
      check: CAN wiring/power to the Launcher motor; ...
 2. [HIGH] Shot NOT ready for 34 s while launcher was spun up; blocking gates: PoseTrusted 27s
 ...
LOOKED CLEAN: battery ok (min 11.9 V while enabled); loop timing / CPU / memory ok; ...
```

- **build / match** come from `BuildConstants/*` and the `DS:*` topics: confirm it is the right log and the right code before anything else.
- **when** is seconds since RIO boot, plus seconds into the enabled period. Match it to what the drive team remembers ("it died right after auto").
- **Severity**: CRITICAL is something that stops the robot (motor disconnect while enabled, brownout, E-stop, log ends while enabled, no movement in auto). HIGH is a subsystem failing its job (setpoint not reached, shot gates closed, vision untrusted, errors in the console). MEDIUM is a warning sign. LOW is noise worth one glance.
- **LOOKED CLEAN** is as important as the suspects: it is the list of things the pit crew does not need to check.

## Detectors (what is checked and on which topics)

|         Area          |                                                          Topics                                                          |                                     Fires when                                      |
|-----------------------|--------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------|
| Driver Station        | `DS:enabled` `DS:autonomous` `DS:test` `DS:estop` `DS:matchNumber` `Match Data/MatchNumber`                                                       | E-stop; disable + re-enable inside a match; auto shorter than 14 s; log ends enabled |
| Motors                | `<Mech>/MotorConnected` `/StatorCurrent` `/Temp` (from `Mechanism.logDiagnostics`)                                       | disconnected ≥ 0.1 s; ≥ 80 A for 1 s; ≥ 80 °C                                       |
| Power                 | `BatteryLogger/BatteryVoltage` `BatteryLogger/Current` `SystemStats/BrownedOut`                                          | ≤ 6.8 V (brownout); dips < 9 V while enabled; > 250 A                               |
| Loop / RIO            | `System/Loop/MaxPeriodMs` `System/Loop/OverrunPercent` `System/CpuPercent` `System/MemAvailableMB` `System/Gc/*` `Scheduler/*` | period ≥ 60 ms; overrun ≥ 5 %; CPU ≥ 92 % for 5 s; < 25 MB free; a phase ≥ 100 ms |
| Stalls                | loop-rate double topics (top three by record count)                                                                      | ≥ 0.3 s with no records while enabled                                               |
| Console / text        | `messages` (console), `Prints`, `Alerts`                                                                                  | errors, exceptions, warnings, overruns, CAN, timeouts, `Telemetry.Fault` names       |
| Setpoints             | `Launcher/CommandedRPM` vs `RPM`, `LauncherTower/*`, `Hood/CommandedDegrees` vs `PositionDegrees`, `Turret/*`, `Turret/TrackingErrorDegrees` | error above tolerance for ≥ 1–1.5 s while enabled                       |
| Counters              | `DyeRotor/RotorStallCount`, `IntakeExtension/Agitate/StalledPulls`, `.../SkewHoldTimeouts`, `CANConfig/FailedCalls`, `FuelIntake/KickerStallLatched`, `CANConfig/BudgetExhausted` | any increase / any true                                     |
| State machine         | `SuperStructure/WantedSuperState` vs `CurrentSuperState`, each `<Mech>/WantedState` vs `SystemState`, `SuperStructure/ShotReady/*` | mismatch ≥ 2 s; `Composite` false while `Launcher/CommandedRPM` > 500, blamed per gate |
| Vision / pose         | `Vision/PoseTrustedForAiming` `Vision/PoseSeedConfirmed` `Vision/SecondsSinceAcceptedEstimate` `Vision/PoseReset/Rejection` `Vision/TurretZero/SlipDegPerMinute` `Swerve/State/Pose` | untrusted > 10 % / 30 % of enabled time; not seeded at auto start; > 5 s without estimate; pose jump ≥ 1 m |
| Auton                 | `Auton/StartPoseErrorMeters` `Auton/StartHeadingErrorDeg` `Auton Warmed Up` `Swerve/State/Pose`                          | ≥ 0.3 m or ≥ 5°; not warmed up; robot moved < 0.2 m in auto                          |

DogLog writes every key under `/Robot/`; the script strips that, so keys read as they do in the Java source.

## Symptom → Where to Look First

|              Drive team says              |                              Findings to lead with                               |
|-------------------------------------------|----------------------------------------------------------------------------------|
| "Robot died / went dead mid-match"        | Log ends while enabled, brownout, disabled mid-match, motor disconnects           |
| "It wouldn't shoot"                       | Shot NOT ready gates (first gate named), launcher/hood setpoint, turret tracking  |
| "Turret was pointing the wrong way"       | Vision untrusted, pose jumped, turret zero slipping, `Turret/MotorConnected`      |
| "Auto did nothing / drove wrong"          | Robot did NOT move in auto, start pose/heading error, not seed-confirmed, not warmed up, `Could not load path planner paths` in console |
| "Intake jammed / stopped"                 | Stall counters, `KickerStallLatched`, stator current ≥ 80 A, `IntakeRoller`/`IntakeKicker` MotorConnected |
| "Robot felt laggy / controls delayed"     | Loop overruns, CPU, GC, code went quiet, loop-time warnings in console            |
| "Nothing looked wrong but score was low"  | LOW findings with `--all`, then hand off to `$wpilog-decode` for the shot rows (`docs/tools/shot-log.md`) |

## Reply Template

Keep the first reply to roughly this shape, then stop:

```
Most likely: <suspect 1> at <when>. Check: <physical thing>.
Also: <suspect 2>, <suspect 3> (one line each).
Ruled out: <LOOKED CLEAN list>.
Log covers <build>, <match>, <enabled windows>. Say the word for a deeper dive on <suspect 1>.
```

## Maintaining This Skill

- When a new key is added to `Mechanism.logDiagnostics`, `SystemLoadMonitor`, or the subsystems' `SystemState`/`WantedState` pairs, add it to the matching table in `triage_wpilog.py` (`SETPOINT_PAIRS`, `STALL_COUNTERS`, `STATE_PAIRS`) and to the table above.
- When a real match log was misdiagnosed, add a detector or adjust a threshold here rather than doing the same manual analysis twice. Verify new key names against `src/main/java` and `docs/tools/logging.md`.
- Thresholds are the constants at the top of `triage_wpilog.py`. Keep them conservative: a triage that cries wolf gets ignored in the pit.
- The reader (`wpilog_fast.py`) decodes primitives, arrays, and `Pose2d`/`Translation2d`/`Rotation2d`/`ChassisSpeeds`. Anything more belongs in `$wpilog-decode`.
