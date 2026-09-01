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
import frc.robot.subsystems.SuperStructure;
import frc.spectrumLib.telemetry.Telemetry;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class Auton {

    public static final EventTrigger autonIntake = new EventTrigger("intake");
    public static final EventTrigger autonShotPrep = new EventTrigger("shotPrep");
    public static final EventTrigger autonShoot = new EventTrigger("shoot");
    public static final EventTrigger autonShootWithIntake = new EventTrigger("shootWithIntake");
    public static final EventTrigger autonClearState = new EventTrigger("clearState");
    public static final EventTrigger autonUnjam = new EventTrigger("unjam");
    public static final EventTrigger autonPoseUpdate = new EventTrigger("poseUpdate");

    private final SendableChooser<Command> pathChooser = new SendableChooser<>();
    private boolean autoMessagePrinted = true;
    private double autonStart = 0;

    /**
     * This method configures the available autonomous routines that can be selected from the
     * SmartDashboard.
     */
    public void setupSelectors() {

        pathChooser.setDefaultOption("Do Nothing", doNothing());

        pathChooser.addOption("OSTBTB Left", OSTBTB(false));
        pathChooser.addOption("OSTBTB Right", OSTBTB(true));
        pathChooser.addOption("OSRIPOFF", OSRIPPOFF(false));

        SmartDashboard.putData("Auto Chooser", pathChooser);
    }

    @SuppressWarnings("unused")
    private SuperStructure robotSuperStructure;

    /**
     * Creates a new Auton instance.
     *
     * @param robotSuperStructure the robotSuperStructure
     */
    public Auton(SuperStructure robotSuperStructure) {
        this.robotSuperStructure = robotSuperStructure;
        setupSelectors(); // runs the command to start the chooser for auto on shuffleboard
        Telemetry.print("Auton Subsystem Initialized");
    }

    /** Init. */
    public void init() {
        Command autonCommand = getAutonomousCommand();

        if (autonCommand != null) {
            CommandScheduler.getInstance().schedule(autonCommand);
            startAutonTimer();
        } else {
            Telemetry.print("No Auton Command Found");
        }
    }

    /** Exit. */
    public void exit() {
        printAutoDuration();
    }

    /** Do nothing. */
    public Command doNothing() {
        return Commands.print("Do Nothing Auto ran").withName("Do Nothing");
    }

    public Command OSTBTB(boolean mirrored) {
        return Commands.sequence(SpectrumAuton("OSTBTB Full", mirrored))
                // the "- Right" and "- Left" is added to the name of the command so that when the
                // visualizer checks the name of the command it can determine whether the auto is
                // mirrored or not and correctly mirror the poses
                .withName("OSTBTB Full - " + (mirrored ? "Right" : "Left"));
    }

    public Command OSRIPPOFF(boolean mirrored) {
        return Commands.sequence(SpectrumAuton("OSRIPPOFF Full", false))
                // the "- Right" and "- Left" is added to the name of the command so that when the
                // visualizer checks the name of the command it can determine whether the auto is
                // mirrored or not and correctly mirror the poses
                .withName("OSRIPPOFF Full - " + (mirrored ? "Right" : "Left"));
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
    /** Spectrum auton. */
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
    /** Follow single path. */
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
    /** Pathfinding command to pose. */
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
    /** Log. */
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
