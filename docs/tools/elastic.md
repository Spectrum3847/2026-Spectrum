# Elastic Dashboard

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

[Elastic](https://github.com/Gold872/elastic-dashboard) is the driver-station dashboard we run during practice and matches. It reads NetworkTables, surfaces alerts and telemetry, gives the operator a clean place to pick autos, and renders the field. It ships with the WPILib installer.

## Our Layout

The layout lives at [`src/main/deploy/elastic-layout.json`](../../src/main/deploy/elastic-layout.json). It deploys to the roboRIO with the rest of the static files via GradleRIO's `frcStaticFileDeploy`, so anyone connecting to the robot can pull the same tabs.

There are six tabs:

**Pre-Match** — FMS info, battery, the turret camera stream, alerts, `Field2d` with the auto path and the three camera pose markers, shot model and hood trim, motor and camera connection boxes, heading-seeded, and the auto chooser. This is what is on screen between matches. It carries one camera stream, not three, on purpose: every MJPEG stream costs radio bandwidth against the 4 Mbps field cap, and this is the tab that is open while the robot sits on the field before auto. The connection boxes and the Field2d markers say whether the chassis cameras are up and seeing tags.

**Match** — `Field2d`, SHOT READY with its two inputs, vision age, distance, launcher RPM, hood angle, shift time, battery, alerts and the current super state. Nothing else.

**Turret** — tracking error and position graphs, the turret camera, slip and trim numbers, ready-to-shoot. The page for turret zero and slip problems.

**Shooting** — launcher RPM and hood position graphs against the shot calculator's wanted values, the feed path RPMs, and the hood trims. The page you stare at when shots are not landing.

**Power** — battery voltage and total current graphs, per-mechanism current bars, follower currents (a dead follower is only visible here), CANivore utilization and energy used.

**Diagnostic** — loop time graph, RIO CPU graph, loop mean and overrun share, GC time, available memory, heap, the scheduler, alerts, camera connections and estimate ages. See [System health alerts](#system-health-alerts).

If you add a widget, edit the layout in Elastic and save it back to the file, and make sure the topic it reads is published by a `Telemetry.logDash` or `logDashAlways` call; plain `Telemetry.log` keys are not on NetworkTables. Spotless leaves JSON alone, so let Elastic round-trip it instead of hand-editing whitespace.

## System Health Alerts

`SystemLoadMonitor` samples the roboRIO once a second and publishes `System/CpuPercent`, `System/MemAvailableMB`, `System/HeapUsedMB`, `System/Gc/MsPerSecond` and the loop period mean, max and overrun share under `System/Loop/`. It raises Driver Station alerts, which show in every Alerts widget:

| alert | condition |
|---|---|
| roboRIO CPU high (warning) | CPU at or above 85 % for 10 s; clears under 80 % |
| Robot loop overrunning (warning) | more than half the loops over 25 ms for 5 s; clears under a quarter |
| Robot loop stalled while enabled (error) | one enabled loop over 200 ms; stays up 10 s |
| GC pause while enabled (warning) | 100 ms or more of collector time in one second while enabled; stays up 10 s |
| roboRIO memory low (warning) | under 24 MB available for 10 s |

On 2026-09-05 the CPU sat at 92 to 95 percent all day and nothing on the dashboard said so. If the CPU alert shows in practice, the fix is less logging, fewer CAN frames or less NetworkTables traffic, not a bigger heap. Thresholds are constants at the top of `SystemLoadMonitor`.

## NetworkTables, In Brief

Elastic talks to the robot over NetworkTables. Anything the robot publishes — `SmartDashboard.put*`, Shuffleboard, or our `Telemetry.logDash` (plain `Telemetry.log` stays in the wpilog unless the `Telemetry/MirrorLogsToNT` switch is on) — is reachable. Widgets bind to a topic like `/SmartDashboard/Field2d` or `/Robot/Initialized`, which is why our log keys use a `Subsystem/Path/Name` hierarchy. It keeps the topic tree navigable.

The reverse direction works too. The auto chooser writes back over NT to a `SendableChooser`. Live-tunable values use `SmartDashboard.getNumber(...)` wrapped by `TuneValue` (see [PID Tuning](pid-tuning.md)).

## Connecting

Install Elastic (WPILib installer is the easy path; releases also up on [GitHub](https://github.com/Gold872/elastic-dashboard/releases) for Linux/macOS). Point it at the robot — `roborio-3847-frc.local` for the real bot, `localhost` for sim — and load the layout from `File → Open Layout`. Once you've connected to a robot once, there's a "Download from robot" option that grabs whatever the RIO has deployed.

On the driver-station laptop, pin Elastic to the same monitor position every match. The match-day team relies on muscle memory, and a relocated widget at the wrong moment is exactly the kind of small problem that ends up costing points.

## A Few Habits

One job per tab. The Pre-Match / Match / Launching split exists so the operator isn't hunting for a widget while a match is running.

Color-code booleans consistently. Most of our `Boolean Box` widgets use green for `true` and red for `false`. The operator's eyes get used to it; mixing colors slows them down.

Use `Field2d` for paths. Publishing a PathPlanner trajectory to `Field2d` lets us preview an auto from Pre-Match without restarting the robot code, which is worth a surprising amount during a hectic afternoon.

## SmartDashboard vs Elastic

Worth a quick clarification because the names overlap. WPILib's built-in `SmartDashboard` is just NetworkTables under the hood; Elastic is a richer client that talks to the same data. Anything `SmartDashboard.put*` publishes is visible from both. We treat SmartDashboard as the fallback (quick tests where editing the layout isn't worth it) and Elastic as the curated view that goes to competitions.
