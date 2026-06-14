# Elastic Dashboard

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

[Elastic](https://github.com/Gold872/elastic-dashboard) is the driver-station dashboard we run during practice and matches. It reads NetworkTables, surfaces alerts and telemetry, gives the operator a clean place to pick autos, and renders the field. It ships with the WPILib installer.

## Our Layout

The layout lives at [`src/main/deploy/elastic-layout.json`](../../src/main/deploy/elastic-layout.json). It deploys to the roboRIO with the rest of the static files via GradleRIO's `frcStaticFileDeploy`, so anyone connecting to the robot can pull the same tabs.

There are five tabs right now:

**Pre-Match** — auto chooser, FMS info, robot-init state, alerts, and `Field2d`. This is what's on screen between matches.

**Match** — what the drivers and operators care about during a match: current state, alliance info, scoring readiness, vision status.

**Launching** — launcher and hood telemetry: wheel velocities, target distance, on-target booleans. The page you stare at when shots aren't landing.

**Diagnostic** — subsystem health, current draw, fault flags. Deeper telemetry for when something's actually broken.

**Git Status** — branch, commit, and build timestamp from `BuildConstants`. The "what's actually running on this robot" tab.

If you add a widget, edit the layout in Elastic and save it back to the file. Spotless leaves JSON alone, so let Elastic round-trip it instead of hand-editing whitespace.

## NetworkTables, In Brief

Elastic talks to the robot over NetworkTables. Anything the robot publishes — `SmartDashboard.put*`, Shuffleboard, or our `Telemetry.log` (which mirrors to NT when configured) — is reachable. Widgets bind to a topic like `/SmartDashboard/Field2d` or `/Robot/Initialized`, which is why our log keys use a `Subsystem/Path/Name` hierarchy. It keeps the topic tree navigable.

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
