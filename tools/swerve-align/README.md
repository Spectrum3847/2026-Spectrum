# Swerve alignment app

A little local web app for zeroing the swerve modules and writing the result straight into the
robot code, so nobody has to copy numbers out of Phoenix Tuner X by hand.

Full walkthrough: [`docs/tools/swerve-alignment.md`](../../docs/tools/swerve-alignment.md).

## Running it

Any of these, from anywhere in the repo:

- Double-click `tools/swerve-align/align-swerve.bat`
  (right-click it → **Send to** → **Desktop (create shortcut)** to make a desktop icon)
- `node tools/swerve-align/server.js`
- `./gradlew alignSwerve`

All three start the same server on <http://127.0.0.1:5817> and open a browser. Press `Ctrl+C` in
the console window to stop it.

## How it fits together

| Piece | Job |
| --- | --- |
| `src/main/java/frc/robot/subsystems/swerve/SwerveAlignment.java` | Publishes raw CANcoder data to NetworkTables under `/Robot/Swerve/Align/` |
| `public/nt4.js` | Read-only NT4 client; the browser talks to the robot directly |
| `public/app.js` | Offset math, the 180° checks, the UI |
| `server.js` | Serves the page, and is the only thing that reads or writes the config `.java` file |

The browser talks to the robot; the Node server talks to the source tree. They never swap roles.

## Notes for whoever maintains this

- **The physical method is alignment pins, not a straight edge.** The modules aren't on a
  rectangle — different front and rear track widths, and the rear modules mount to the angled
  plates of the hexagonal frame — so there's no pair of wheels a bar can sit flat against. MK5n
  modules pin to themselves, which sidesteps the frame entirely. The "Modules to align"
  checkboxes exist for the same reason: aligning one module at a time is normal here.
- **The theme is the spectrum3847.org palette** (`--p800: #3c0060` chrome on white, `--purple:
  #6b1199` accents). Plus Jakarta Sans and Outfit are named in the font stack but deliberately
  never fetched — a blocking webfont request in a pit with no internet costs a DNS timeout before
  anything paints.

- **Which file it writes** is the `TARGET_CONFIG` constant at the top of `server.js`. It is
  hardcoded to `OM2026.java` on purpose — `Robot.java` currently selects that config for every
  RoboRIO. Point it somewhere else by changing that one line.
- **The server binds to `127.0.0.1` only.** It can rewrite source files, so it must not be
  reachable from the pit network.
- **No dependencies, no `package.json`**, matching `scripts/wpilog.js`. Node 18 or newer.
- **`node server.js --self-test`** round-trips the config parser and writer against the real file
  (in memory) plus a couple of shapes it has to survive. Run it after touching the parsing code.
- The written values are rounded to 1/4096 of a rotation, the CANcoder's own resolution, which is
  why they come out looking like the hand-entered constants already in the configs.
- The emitted call is formatted the way `googleJavaFormat().aosp()` would format it, so
  `./gradlew spotlessCheck` stays green without anyone running `spotlessApply` afterwards.
