# Elastic dashboard

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

[Elastic](https://github.com/Gold872/elastic-dashboard) is the driver-station dashboard we run during practice and matches. It reads NetworkTables, surfaces alerts and telemetry, gives the operator a clean place to pick autos, and renders the field. It ships with the WPILib installer.

WPILib's built-in `SmartDashboard` is just NetworkTables with a fixed set of widgets, which is why the two names overlap. Anything published through `SmartDashboard.put*` is visible from both. We treat `SmartDashboard` as the fallback for a quick test where editing a layout is not worth it, and Elastic as the curated view that goes to competitions.

## Our layout

The layout lives at [`src/main/deploy/elastic-layout.json`](../../src/main/deploy/elastic-layout.json). It deploys to the roboRIO with the rest of the static files, so anyone connecting to the robot can pull the same tabs, and Elastic can download whatever the RIO has straight from a previously connected machine.

Read the file for what is on each tab. The reason the split is roughly pre-match, match, turret, shooting, power and diagnostics is that one job per tab keeps the operator from hunting for a widget while a match is running. During a match itself, only the Match tab matters, and the split keeps it to the field view, the shot-ready state and its inputs, the numbers the operator actually calls, and the alerts.

Three things about the layout that are decisions rather than trivia:

* **Pre-Match carries one camera stream, not three.** Every MJPEG stream costs radio bandwidth against the 4 Mbps field cap, and Pre-Match is the tab that is open while the robot sits on the field before auto. If you add a second stream there, think about what you are spending it on.

* **The shooting trims persist across a redeploy and a power cycle.** So the operator should read them before a match rather than discover them. The shots-logged counter on that tab is the signal that records are being written at all. See [Shot Records and Trim Events](shot-log.md).

* **Diagnostics is the system health tab, and it is the one to watch when the robot feels slow.** Loop time, RIO CPU, loop mean and overrun share, GC time, available memory and heap, the scheduler, alerts, and camera connection and estimate ages.

If you add a widget, edit the layout in Elastic and save it back to the file rather than hand-editing the JSON, and make sure the topic it reads is published by a `Telemetry.logDash` or `logDashAlways` call. A plain `Telemetry.log` key is not on NetworkTables. Spotless leaves JSON alone, so letting Elastic round-trip the file does not fight the formatter.

## System health alerts

`SystemLoadMonitor` samples the roboRIO once a second, publishes the CPU, available memory, heap used and collector time under `System/`, and the loop period mean, max and overrun share under `System/Loop/`. It raises Driver Station alerts, which show in every Alerts widget anywhere in the layout. The thresholds, and the reasoning behind each one, are constants at the top of that class.

On 2026-09-05 the CPU sat at 92 to 95 percent all day and nothing on the dashboard said so. When the CPU alert shows in practice, the fix is less logging, fewer CAN frames or less NetworkTables traffic, not a bigger heap. A bigger heap does not make the work smaller, and on the RIO it trades against something else.

## Connecting

Install Elastic. The WPILib installer is the easy path; releases are also on [GitHub](https://github.com/Gold872/elastic-dashboard/releases) for Linux and macOS. Point it at the robot, `roborio-3847-frc.local` for the real bot and `localhost` for sim, then load the layout from `File`, `Open Layout`.

On the driver-station laptop, pin Elastic to the same monitor position every match. The match-day team relies on muscle memory, and a relocated widget at the wrong moment is exactly the kind of small problem that costs points.

## Conventions worth keeping

* **Colour-code booleans consistently.** Most of our `Boolean Box` widgets are green for true and red for false. The operator's eyes get used to it and mixing colours slows them down.

* **Publish paths to `Field2d`.** That lets an auto be previewed from Pre-Match without restarting the robot code, which is worth a surprising amount during a hectic afternoon.

* **Publish state changes with `logStateDash`, not `logDash`.** A widget that samples a slow-tier key can miss a state that only held for one loop. `Telemetry` has a tier for exactly this; see [Logging and Data Analysis](logging.md).

## See also

[Logging and Data Analysis](logging.md) for which tier a key has to be in to reach the dashboard. [Shot Records and Trim Events](shot-log.md) for the keys on the shooting tab.
