# Spectrum robot app

A local web app for the **2026 offseason robot** (`OM2026`). Controls reference, log sync,
power analysis, and CAN-bus health, in one place, running on your laptop.

```bash
cd tools/robot-app
npm install          # once, needs internet
npm start            # builds and serves on http://localhost:5801
```

After `npm install`, nothing needs the internet again — it runs in the queue line, in the pit, on
a field with no signal. Only pulling logs off the robot needs a network, and that network is the
robot's own.

For development with hot reload:

```bash
npm run dev          # Vite on 5173, API proxied to the Express server on 5801
```

## What's here

| Page | What it does |
| --- | --- |
| **Pilot** / **Operator** | Button maps with a live controller diagram, one layer per modifier chord, including the disabled-mode pit controls that are otherwise only findable by reading `Robot.java`. |
| **Logs** | Finds the robot, lists `.wpilog` files on it, pulls them into the team logs repo, and indexes headline numbers into a committed manifest. |
| **Power** | Per-motor current against its configured limit, how often each motor is pinned there, battery sag and pack internal resistance, energy per mechanism, and a main-breaker thermal simulation. |
| **CAN Bus** | Bus utilization, error counters heading for bus-off, motors that stopped answering, and the device inventory. |
| **Swerve Align** | A slot. See *Adding a page*. |

## The two data files

Both are hand-authored and both are checked against the Java by `npm run check` (and by
`./gradlew check`).

**`data/controls.json`** — every binding in `Robot.configureBindings()`.

Hand-written rather than generated on purpose. The LT/RT launch behaviour lives inside
`Commands.either(...)` conditions, so the trigger expression alone understates what the robot
does; a generator would produce confident, wrong documentation. The drift check enforces that
every `pilot.X` / `operator.X` the Java binds appears here and vice versa.

**`data/robot-profile.json`** — CAN ids, buses, current limits, follower relationships.

This one is load-bearing. **Current limits are never logged.** They are compiled into the Java
config classes, so without this file no analyzer can draw a limit line or say "this motor was at
its ceiling 17% of the match". The drift check verifies every limit against its Java source.

## Reading the analysis honestly

A few things about this robot's telemetry will mislead you if you don't know them. The Power page
lists them all at the bottom; the ones that bite hardest:

- **Per-motor currents are leader-only.** `Launcher/StatorCurrent`, `IntakeRoller/*` and
  `LauncherTower/*` exclude their follower, so true draw is roughly double. The
  `BatteryLogger/Current/Mechanisms/*` column does include followers.
- **`MotorConnected` exists for only three motors** (Turret, Launcher, Hood) and only checks the
  leader. A dead Launcher Front Right (CAN 16) is invisible in the log. That is why the CAN page
  also infers dropouts from motors reporting 0 V and 0 A while a setpoint is pending — the
  signature that found the dead hood on 2026-09-04.
- **The roboRIO CAN bus has no health metrics at all.** The intake roller pair (CAN 6 and 7) lives
  there, so a fault on that bus shows up only as a frozen trace.
- **`Scheduler/*` is in seconds**, not milliseconds.
- **Brownout is not logged** and the threshold is set to 4.6 V, well under the 6.8 V default, so
  the RIO holds on far longer than stock. Sag has to be read off the voltage trace.

## Logs repo

Synced logs land in a clone of
[Spectrum3847/2026-Robot-Logs](https://github.com/Spectrum3847/2026-Robot-Logs), expected as a
sibling of this repository:

```bash
git clone https://github.com/Spectrum3847/2026-Robot-Logs ../../../2026-Robot-Logs
```

Point somewhere else with `config.local.json` (gitignored):

```json
{ "logsRepo": { "path": "/absolute/path/to/2026-Robot-Logs" } }
```

`.wpilog` files are gitignored there by default — a season of match logs is tens of gigabytes and
git has no good answer for that. What *is* committed is `manifest.json`, carrying each log's
duration, enabled time, min voltage, peak current, energy, loop overrun rate and peak CAN load. So
log history survives in the repo even when the binaries don't. Use **pin** on the Logs page to
force a specific log into git when it documents something worth keeping.

## Finding the robot

Probed in this order, and whichever answers on port 22 first wins:

| Address | What it is |
| --- | --- |
| `10.85.15.2` | Team 8515 over the radio — the number this robot's radio is configured for |
| `roborio-8515-frc.local` | Same, over mDNS |
| `10.38.47.2` | Team 3847 over the radio |
| `roborio-3847-frc.local` | Same, over mDNS |
| `172.22.11.2` | USB, works with no radio at all |

Logs are read from `/U/logs` (the USB stick) and `/home/lvuser/logs` (internal flash).

## Adding a page

Every directory under `client/pages/` containing an `index.html` becomes its own Vite entry point
automatically — no build config to touch.

```
client/pages/my-page/
  index.html      <main id="app"></main> plus <script type="module" src="./main.js">
  main.js
```

In `main.js`:

```js
import "../../styles.css";                    // cards, tables, stat tiles, tags
import { mountHeader, el, api } from "../../lib/ui.js";
mountHeader();                                // shared nav + robot status pill
```

Add a nav entry in `client/lib/ui.js` (`PAGES`). Useful pieces:

| Module | For |
| --- | --- |
| `lib/wpilog.js` | `.wpilog` parser, works in the browser and in Node |
| `lib/log-model.js` | Normalizes a log across both log eras; discovers mechanisms; enabled windows |
| `lib/charts.js` | Chart.js defaults, time charts, limit lines, enabled-time shading, decimation |
| `lib/log-loader.js` | The log picker used by Power and CAN |
| `/api/robot/probe` | Which RIO address is reachable |
| `/api/logs` | Synced logs and their manifest entries |

For live robot data, talk NT4 straight from the browser at
`ws://<robot-host>:5810/nt/<client-id>`. The server doesn't need to be in the middle.

## Layout

```
server/          Express API: robot discovery, SSH log transfer, manifest, git
  lib/config.js  config.default.json + config.local.json + --port
  lib/robot.js   address probing, SSH listing, SFTP download
  lib/summary.js headline numbers extracted at sync time
  lib/manifest.js manifest read/write, commit and push
client/          Vite frontend, one entry per page
  lib/           shared parser, log model, charts, UI helpers
  pages/         one directory per page
data/            controls.json, robot-profile.json
scripts/         check-drift.mjs
test/            node:test coverage of the analysis maths
```

## Commands

```bash
npm start        # build + serve
npm run dev      # hot reload
npm test         # analysis maths
npm run check    # drift check + tests
```

`./gradlew check` runs the drift check too, and skips it when Node isn't on the PATH.
