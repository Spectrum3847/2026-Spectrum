package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotStates;
import frc.robot.State;
import frc.robot.swerve.SwerveStates;
import frc.spectrumLib.SpectrumState;
import frc.spectrumLib.Telemetry;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class Auton {

    public static final SpectrumState autonLaunching = new SpectrumState("AutonLaunching");

    public static final EventTrigger autonIntake = new EventTrigger("intake");
    public static final EventTrigger autonShotPrep = new EventTrigger("shotPrep");
    public static final EventTrigger autonShoot = new EventTrigger("shoot");
    public static final EventTrigger autonClearState = new EventTrigger("clearState");
    public static final EventTrigger autonUnjam = new EventTrigger("unjam");
    public static final EventTrigger autonPoseUpdate = new EventTrigger("poseUpdate");

    private final SendableChooser<Command> pathChooser = new SendableChooser<>();
    private boolean autoMessagePrinted = true;
    private double autonStart = 0;

    private final double SECOND_MAN_DELAY = 2.0;
    private final double OPTIONAL_DELAY = 1.0;

    /**
     * This method configures the available autonomous routines that can be selected from the
     * SmartDashboard.
     */
    public void setupSelectors() {

        pathChooser.setDefaultOption("Do Nothing", doNothing());

        pathChooser.addOption("TBTB Left", TBTB(false));
        pathChooser.addOption("TBTB Right", TBTB(true));

        pathChooser.addOption("TBTT Left", TBTT(false));
        pathChooser.addOption("TBTT Right", TBTT(true));

        pathChooser.addOption("TTTT Left", TTTT(false));
        pathChooser.addOption("TTTT Right", TTTT(true));

        pathChooser.addOption("BBBB Left", BBBB(false));
        pathChooser.addOption("BBBB Right", BBBB(true));

        pathChooser.addOption("Optional - Left TBT", optional_TBT(false));
        pathChooser.addOption("Optional - Right TBT", optional_TBT(true));

        pathChooser.addOption("Optional - Left BBB", optional_BBB(false));
        pathChooser.addOption("Optional - Right BBB", optional_BBB(true));

        pathChooser.addOption("2nd Man - TBTB Left", secondMan_TBTB(false));
        pathChooser.addOption("2nd Man - TBTB Right", secondMan_TBTB(true));

        pathChooser.addOption("2nd Man - BBD Left", secondMan_BBD(false));
        pathChooser.addOption("2nd Man - BBD Right", secondMan_BBD(true));

        SmartDashboard.putData("Auto Chooser", pathChooser);
    }

    public Auton() {
        setupSelectors(); // runs the command to start the chooser for auto on shuffleboard
        Telemetry.print("Auton Subsystem Initialized");
    }

    public void init() {
        Command autonCommand = getAutonomousCommand();

        if (autonCommand != null) {
            CommandScheduler.getInstance().schedule(autonCommand);
            startAutonTimer();
        } else {
            Telemetry.print("No Auton Command Found");
        }
    }

    public void exit() {
        printAutoDuration();
    }

    public Command doNothing() {
        return Commands.print("Do Nothing Auto ran").withName("Do Nothing");
    }

    public Command prepThanLaunch() {
        return Commands.deadline(
                Commands.sequence(
                        autonLaunching.setTrue(),
                        RobotStates.applyState(State.LAUNCHER_TRACK),
                        Commands.waitSeconds(0.5),
                        RobotStates.applyState(State.LAUNCHER_TRACK_WITH_LAUNCH),
                        Commands.waitSeconds(2.5),
                        RobotStates.applyState(State.IDLE),
                        autonLaunching.setFalse()),
                SwerveStates.autonAimAtTarget());
    }

    public Command secondMan_TBTB(boolean mirrored) {
        return Commands.sequence(
                        Commands.waitSeconds(SECOND_MAN_DELAY),
                        SpectrumAuton("2nd-TBTB 1", mirrored),
                        prepThanLaunch(),
                        SpectrumAuton("2nd-TBTB 2", mirrored))
                // the "- Right" and "- Left" is added to the name of the command so that when the
                // visualizer checks the name of the command it can determine whether the auto is
                // mirrored or not and correctly mirror the poses
                .withName("2nd-TBTB Full - " + (mirrored ? "Right" : "Left"));
    }

    public Command secondMan_BBD(boolean mirrored) {
        return Commands.sequence(
                        Commands.waitSeconds(SECOND_MAN_DELAY),
                        SpectrumAuton("2nd-BBD 1", mirrored),
                        prepThanLaunch(),
                        SpectrumAuton("2nd-BBD 2", mirrored))
                // the "- Right" and "- Left" is added to the name of the command so that when the
                // visualizer checks the name of the command it can determine whether the auto is
                // mirrored or not and correctly mirror the poses
                .withName("2nd-BBD Full - " + (mirrored ? "Right" : "Left"));
    }

    public Command optional_TBT(boolean mirrored) {
        return Commands.sequence(
                        SpectrumAuton("Option TBT 1", mirrored),
                        Commands.waitSeconds(OPTIONAL_DELAY),
                        SpectrumAuton("Option TBT 2", mirrored),
                        prepThanLaunch(),
                        SpectrumAuton("Option TBT 3", mirrored))
                // the "- Right" and "- Left" is added to the name of the command so that when the
                // visualizer checks the name of the command it can determine whether the auto is
                // mirrored or not and correctly mirror the poses
                .withName("Option TBT Full - " + (mirrored ? "Right" : "Left"));
    }

    public Command optional_BBB(boolean mirrored) {
        return Commands.sequence(
                        SpectrumAuton("Option BBB 1", mirrored),
                        Commands.waitSeconds(OPTIONAL_DELAY),
                        SpectrumAuton("Option BBB 2", mirrored),
                        prepThanLaunch(),
                        SpectrumAuton("Option BBB 3", mirrored))
                // the "- Right" and "- Left" is added to the name of the command so that when the
                // visualizer checks the name of the command it can determine whether the auto is
                // mirrored or not and correctly mirror the poses
                .withName("Option BBB Full - " + (mirrored ? "Right" : "Left"));
    }

    public Command TBTB(boolean mirrored) {
        return Commands.sequence(
                        SpectrumAuton("TBTB 1", mirrored),
                        prepThanLaunch(),
                        SpectrumAuton("TBTB 2", mirrored),
                        prepThanLaunch(),
                        SpectrumAuton("TBTB 3", mirrored))
                // the "- Right" and "- Left" is added to the name of the command so that when the
                // visualizer checks the name of the command it can determine whether the auto is
                // mirrored or not and correctly mirror the poses
                .withName("TBTB Full - " + (mirrored ? "Right" : "Left"));
    }

    public Command TBTT(boolean mirrored) {
        return Commands.sequence(
                        SpectrumAuton("TBTT 1", mirrored),
                        prepThanLaunch(),
                        SpectrumAuton("TBTT 2", mirrored),
                        prepThanLaunch(),
                        SpectrumAuton("TBTT 3", mirrored))
                // the "- Right" and "- Left" is added to the name of the command so that when the
                // visualizer checks the name of the command it can determine whether the auto is
                // mirrored or not and correctly mirror the poses
                .withName("TBTT Full - " + (mirrored ? "Right" : "Left"));
    }

    public Command TTTT(boolean mirrored) {
        return Commands.sequence(
                        SpectrumAuton("TTTT 1", mirrored),
                        prepThanLaunch(),
                        SpectrumAuton("TTTT 2", mirrored),
                        prepThanLaunch())
                // the "- Right" and "- Left" is added to the name of the command so that when the
                // visualizer checks the name of the command it can determine whether the auto is
                // mirrored or not and correctly mirror the poses
                .withName("TTTT Full - " + (mirrored ? "Right" : "Left"));
    }

    public Command BBBB(boolean mirrored) {
        return Commands.sequence(
                        SpectrumAuton("BBBB 1", mirrored),
                        prepThanLaunch(),
                        SpectrumAuton("BBBB 2", mirrored),
                        prepThanLaunch())
                // the "- Right" and "- Left" is added to the name of the command so that when the
                // visualizer checks the name of the command it can determine whether the auto is
                // mirrored or not and correctly mirror the poses
                .withName("BBBB Full - " + (mirrored ? "Right" : "Left"));
    }

    /**
     * Creates a SpectrumAuton command sequence.
     *
     * <p>This method generates a command sequence that first waits for 0.01 seconds and then
     * executes a PathPlannerAuto command with the specified autonomous routine name.
     *
     * @param autoName the name of the autonomous routine to execute
     * @param mirrored whether the autonomous routine should be mirrored
     * @return a Command that represents the SpectrumAuton sequence
     */
    public Command SpectrumAuton(String autoName, boolean mirrored) {
        Command autoCommand = new PathPlannerAuto(autoName, mirrored);
        return Commands.waitSeconds(0.01).andThen(autoCommand).withName(autoName);
    }

    public Command SpectrumAuton(String autoName, boolean mirrored, double duration) {
        Command autoCommand = new PathPlannerAuto(autoName, mirrored);
        return Commands.waitSeconds(0.01)
                .andThen(autoCommand)
                .withTimeout(duration)
                .withName(autoName);
    }

    /**
     * Retrieves the autonomous command selected on the shuffleboard.
     *
     * @return the selected autonomous command if one is chosen; otherwise, returns a PrintCommand
     *     indicating that the autonomous command is null.
     */
    public Command getAutonomousCommand() {
        Command auton = pathChooser.getSelected(); // sees what auto is chosen on shuffleboard
        if (auton != null) {
            return auton; // checks to make sure there is an auto and if there is it runs an auto
        } else {
            return new PrintCommand(
                    "*** AUTON COMMAND IS NULL ***"); // runs if there is no auto chosen, which
            // shouldn't happen because of the default
            // auto set to nothing which still runs
            // something
        }
    }

    /** This method is called in AutonInit */
    public void startAutonTimer() {
        autonStart = Timer.getFPGATimestamp();
        autoMessagePrinted = false;
    }

    /** Called at AutonExit and displays the duration of the auton command Based on 6328 code */
    public void printAutoDuration() {
        Command autoCommand = getAutonomousCommand();
        if (autoCommand != null) {
            if (!autoCommand.isScheduled() && !autoMessagePrinted) {
                if (DriverStation.isAutonomousEnabled()) {
                    Telemetry.print(
                            String.format(
                                    "*** Auton finished in %.2f secs ***",
                                    Timer.getFPGATimestamp() - autonStart));
                } else {
                    Telemetry.print(
                            String.format(
                                    "*** Auton CANCELLED in %.2f secs ***",
                                    Timer.getFPGATimestamp() - autonStart));
                }
                autoMessagePrinted = true;
            }
        }
    }

    public static Command followSinglePath(String pathName) {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);

            // Create a path following command using AutoBuilder. This will also trigger event
            // markers.
            return AutoBuilder.followPath(path);
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
        }
        return new PrintCommand("ERROR LOADING PATH");
    }

    public static Command pathfindingCommandToPose(
            double xPos, double yPos, double rotation, double vel, double accel) {
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        Pose2d targetPose = new Pose2d(xPos, yPos, Rotation2d.fromDegrees(rotation));

        // Create the constraints to use while pathfinding
        PathConstraints constraints =
                new PathConstraints(
                        vel, accel, Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand =
                AutoBuilder.pathfindToPoseFlipped(
                        targetPose, constraints, 0.0 // Goal end velocity in meters/sec
                        );

        return pathfindingCommand;
    }
    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
