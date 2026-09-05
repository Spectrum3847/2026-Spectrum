# Spectrum robot app

A local web app for the **2026 offseason robot** (`OM2026`). Controls reference, log sync,
power analysis, and CAN-bus health, in one place, running on your laptop.

```bash
cd tools/robot-app
npm install          # once, needs internet
npm start            # builds and serves on http://localhost:5801
```

On Windows, double-click `start.bat` instead — it installs, builds and opens the browser.
`align-swerve.bat` does the same but lands on the alignment page, for the desktop shortcut people
already have. `./gradlew robotApp` and `./gradlew alignSwerve` do the same from Gradle.

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
| **Swerve Align** | Pin the modules, read the CANcoders live over NT4, and write the offsets into the robot config file. |

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

## Swerve alignment

The alignment page is the one part of the app that **writes to the source tree**: it rewrites the
four numbers in `swerve.configEncoderOffsets(...)` in `src/main/java/frc/robot/configs/OM2026.java`
and nothing else. That is why the server binds to `127.0.0.1` — nothing on the pit network should
be able to ask a laptop to edit robot code. Point it at a different robot with `swerveAlign.targetConfig`
in `config.local.json`.

The split is strict: **the browser talks to the robot, the server talks to the source tree, and
they never swap roles.** `client/lib/nt4.js` is a read-only NT4 client that connects straight from
the browser to `ws://<robot>:5810`; `server/lib/swerve-config.js` is the only thing that touches
the `.java` file.

The parser handles the arithmetic already in the configs (`-0.23046875 - 0.25`), rounds to the
CANcoder's own 1/4096 resolution, and emits the call formatted the way `googleJavaFormat().aosp()`
would, so `./gradlew spotlessCheck` stays green without anyone running `spotlessApply` afterwards.
`test/swerve-config.test.mjs` round-trips all of that against the real config file plus the other
call shapes it has to survive.

Robot side: `SwerveAlignment.java` publishes raw CANcoder data under `/Robot/Swerve/Align/`.
The full walkthrough is [`docs/tools/swerve-alignment.md`](../../docs/tools/swerve-alignment.md).

## Theming

Colors come from the team site (`spectrum3847.org`, `src/styles/custom.css`): deep purple
`#3C0060` for the nav, `#6B1199` as the working accent, white surfaces, `#E9DDF7` borders,
`#1F1B23` text, with the site's Plus Jakarta Sans / Outfit pairing. The fonts are self-hosted
through `@fontsource-variable` rather than Google's CDN, so brand typography survives having no
internet.

Everything is a CSS custom property in `client/styles.css`. A dark variant lives under
`:root[data-theme="dark"]` for pit and queue-line use — the same hues re-stepped against a
deep-purple surface, not an inverted light theme. The moon/sun button in the nav toggles it and
remembers the choice per browser.

Chart colors are read from those same properties at draw time, so charts re-skin with the toggle.
The eight categorical series slots were validated against each surface for lightness band, chroma,
colorblind separation, normal-vision separation and 3:1 contrast; brand `#6B1199` is too dark to
be a series color on white, so slot 1 is the nearest passing step, `#7E22CE`. Slots are assigned
in fixed order and never cycled — a ninth series on one chart means splitting the chart.

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
| `lib/nt4.js` | Read-only NT4 client; connects the browser straight to the robot |
| `/api/robot/probe` | Which RIO address is reachable |
| `/api/logs` | Synced logs and their manifest entries |

For live robot data, import `lib/nt4.js` and talk NT4 straight from the browser — the server does
not need to be in the middle. The Swerve Align page is the worked example.

## Layout

```
server/          Express API: robot discovery, SSH log transfer, manifest, git
  lib/config.js  config.default.json + config.local.json + --port
  lib/robot.js   address probing, SSH listing, SFTP download
  lib/summary.js headline numbers extracted at sync time
  lib/manifest.js manifest read/write, commit and push
  lib/swerve-config.js reads and rewrites the encoder offsets in the robot config
client/          Vite frontend, one entry per page
  lib/           shared parser, log model, charts, UI helpers
  pages/         one directory per page
data/            controls.json, robot-profile.json
scripts/         check-drift.mjs
test/            node:test coverage of the analysis maths
```

## When a page shows the wrong content

The Logs page once did this for a reason worth knowing: the repo root's `.gitignore` has `logs/`
for robot log files, and it silently swallowed `client/pages/logs/`. The page worked for anyone
who had it on disk and 404'd for everyone who cloned. `tools/robot-app/.gitignore` re-includes it,
and `scripts/check-drift.mjs` now fails if any page directory is untracked, so it cannot recur
quietly. If you add a page and the drift check complains, run the `git check-ignore -v` command it
prints — a root ignore rule is probably eating it.

Otherwise, if a nav tab lands on the home page, `dist/` is out of date — the page you clicked was built into
`dist/` at some earlier point and no longer matches `client/`. The server now says so instead of
quietly serving the home page: the startup banner prints `client STALE` or `client INCOMPLETE`,
and the page itself returns a message naming what to run.

```bash
npm start          # rebuild and serve
```

`npm run serve` deliberately skips the build, so it is the one way to end up serving a stale
`dist/`. Use `npm start` unless you know you want that.

## Commands

```bash
npm start        # build + serve
npm run dev      # hot reload
npm test         # analysis maths
npm run check    # drift check + tests
```

`./gradlew check` runs the drift check too, and skips it when Node isn't on the PATH.
