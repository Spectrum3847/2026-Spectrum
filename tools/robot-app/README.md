# Spectrum robot app

A local web app for the 2026 offseason robot (`OM2026`): controls reference, log sync, power and
CAN-bus analysis, swerve alignment and camera setup. It runs on your laptop and needs no internet
after `npm install`.

## Running it

```bash
cd tools/robot-app
npm install          # once, needs internet
npm start            # builds and serves on http://localhost:5801
```

On Windows you can double-click `start.bat` instead, or `align-swerve.bat` to open straight to
the alignment page. From Gradle: `./gradlew robotApp` or `./gradlew alignSwerve`.

## Pages

|           Page           |                                     What it does                                      |
|--------------------------|---------------------------------------------------------------------------------------|
| **Pilot** / **Operator** | Button maps for each controller, including the disabled-mode pit controls.            |
| **Logs**                 | Finds the robot, pulls its `.wpilog` files, and uploads them to the log site.         |
| **Turret**               | Whether the turret reached its command and whether it slipped.                        |
| **Power**                | Per-motor current against its limit, battery sag, energy per mechanism.               |
| **CAN Bus**              | Bus utilization, error counters, and motors that stopped answering.                   |
| **Swerve Align**         | Reads the CANcoders and writes the module offsets into the robot config.              |
| **Cameras**              | Checks each Limelight's mount, tunes exposure, and writes the fixes to `Vision.java`. |

Swerve Align and Cameras edit files in this repo, so the server only listens on `127.0.0.1`.
Walkthroughs: [swerve alignment](../../docs/tools/swerve-alignment.md),
[vision](../../docs/tools/vision.md).

## Log uploads

The Logs page pulls `.wpilog` files off the robot into `rio-logs/` at the repo root, then uploads
them to the team log site, [robotlog.spectrum3847.org](https://robotlog.spectrum3847.org). Files
already on the site are skipped.

Uploading runs the import script of
[WPI-Log-Analizer](https://github.com/Spectrum3847/WPI-Log-Analizer), cloned next to this
repository:

```bash
git clone https://github.com/Spectrum3847/WPI-Log-Analizer ../../../WPI-Log-Analizer
```

The first time, the Logs page asks for the upload key (a mentor has it). It is saved in your home
folder, never in a repo: `%APPDATA%\spectrum-logs\upload_key` on Windows,
`~/.config/spectrum-logs/upload_key` on macOS and Linux. A `LOG_SITE_KEY` environment variable
overrides it.

The robot is found at `10.85.15.2`, `10.38.47.2`, their `roborio-<team>-frc.local` names, or
`172.22.11.2` over USB.

## Adding a page

Each directory under `client/pages/` with an `index.html` becomes its own page; no build config
to change. Add it to the nav in `client/lib/ui.js` (`PAGES`).

## Commands

```bash
npm start        # build + serve
npm run dev      # hot reload
npm test         # analysis tests
npm run check    # drift check against the Java + tests
```

`data/controls.json` and `data/robot-profile.json` are written by hand. `npm run check` compares
them against the Java, and every page shows a banner when they have fallen behind.

If a nav tab lands on the home page, `dist/` is stale; run `npm start` to rebuild.
