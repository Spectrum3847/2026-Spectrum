# Elastic dashboard

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

[Elastic](https://github.com/Gold872/elastic-dashboard) is the driver station dashboard we run at practice and at matches. It reads NetworkTables, gives the operator the auto chooser, and renders the field. It ships with the WPILib installer.

## Our layout

The layout is [`src/main/deploy/elastic-layout.json`](../../src/main/deploy/elastic-layout.json). It deploys to the roboRIO with the rest of `src/main/deploy`, so anyone who plugs into the robot gets the same tabs we do. Edit it in Elastic and save back to that file. It is JSON and the formatter does not touch it, so let Elastic write it rather than cleaning up whitespace by hand.

**Pre-Match** is the tab on screen between matches and it owns the chooser. **Git Status** answers "which build is actually on this robot": it reads the build stamp the robot publishes at startup.

## Most of the layout is bound to a prefix nothing publishes

This one costs an afternoon if you do not know it. Nearly every widget in the layout binds to `/Robot/<key>`, for example `/Robot/Launcher/RPM`. `Telemetry.log("Launcher/RPM", ...)` publishes that key under `/DogLog/`, because that is the NetworkTables table DogLog writes into. Nothing in this repo publishes anything under `/Robot/`.

The symptom is a dashboard where the FMS panel, the field, the running-commands widget, the alerts, the chooser, and the camera streams all work, and every numeric readout sits blank. The widgets that do have data are the `/SmartDashboard/...` ones, `/CameraPublisher/...`, and `/FMSInfo`, because those come from the Driver Station and from `SmartDashboard.put*` rather than from DogLog. Only a handful of `SmartDashboard` puts exist, so that is a short list.

To see what is really there, open a NetworkTables browser, AdvantageScope or the sim GUI, and look under `/DogLog/`. Then either repoint the widgets in Elastic or accept that for live values AdvantageScope and the `.wpilog` file are the tools, and Elastic is for the chooser and the field.

## Connecting

Point Elastic at the robot, `roborio-3847-frc.local` for the real bot and `localhost` in sim, then **File, then Open Layout**.

If your copy and the robot's have drifted, the layout menu's **Download from robot** grabs whatever the RIO has deployed. The robot serves the whole `deploy/` directory over HTTP on port 5800, so `http://<rio-ip>:5800/elastic-layout.json` opens in any browser on the network, which saves you when you are not sitting at the driver station laptop.

Pin Elastic to the same monitor position before every match. The operator should never be hunting for a widget mid-match, and a relocated widget at the wrong moment is exactly the kind of small thing that costs points.

## Habits worth keeping

One job per tab, so nothing important is buried under something else.

Consistent colors for booleans. Most of ours are green for true and red for false. Mixed colors on the same screen cost the operator real time, and the habit only works if it is uniform.

Preview the auto on `Field2d` while disabled, before you enable, rather than finding out on the field.

## See also

* [Logging](logging.md) for what `Telemetry` publishes and the key naming convention.
* [PID Tuning](pid-tuning.md) for the live tunables you can edit from here.
