package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.WantedSuperState;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.telemetry.Telemetry.PrintPriority;
import java.util.LinkedHashSet;
import java.util.Set;

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

    /** How long {@link #launch()} holds the launch before idling. */
    private static final double LAUNCH_SECONDS = 2.5;

    /**
     * This method configures the available autonomous routines that can be selected from the
     * SmartDashboard.
     */
    public void setupSelectors() {

        pathChooser.setDefaultOption("Do Nothing", doNothing());

        pathChooser.addOption("Double Swipe Left", single("OSTBTB FULL", false));
        pathChooser.addOption("Double Swipe Right", single("OSTBTB FULL", true));
        pathChooser.addOption("Single Swipe with Depot Left", single("OSRIPPOFF FULL", false));
        pathChooser.addOption("Single Swipe with Depot Right", single("OSRIPPOFF FULL", true));
        pathChooser.addOption("2nd Double Swipe Left", TWOMANOSTBTB(false));
        pathChooser.addOption("2nd Double Swipe Right", TWOMANOSTBTB(true));
        pathChooser.addOption("Center 1 Swipe Left", OSCENT(false));
        pathChooser.addOption("Center 1 Swipe Right", OSCENT(true));
        pathChooser.addOption("Center to Depot Left", single("OSCENTOT FULL", false));
        pathChooser.addOption("Center to Depot Right", single("OSCENTOT FULL", true));
        pathChooser.addOption(
                "Single Swipe with Depot Cutoff Left", single("OSRIPOFF CUTOFF", false));
        pathChooser.addOption(
                "Single Swipe with Depot Cutoff Right", single("OSRIPOFF CUTOFF", true));
        pathChooser.addOption(
                "Double Swipe 1 1/2 Left", single("OSRIPOFF DOUBLE SWIPE FULL", false));
        pathChooser.addOption(
                "Double Swipe 1 1/2 Right", single("OSRIPOFF DOUBLE SWIPE FULL", true));

        SmartDashboard.putData("Auto Chooser", pathChooser);
    }

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
        CommandScheduler.getInstance().schedule(getAutonomousCommand());
        autonStart = Timer.getFPGATimestamp();
        autoMessagePrinted = false;
    }

    /** Exit. Prints how long the auto ran, once, based on 6328's code. */
    public void exit() {
        if (!getAutonomousCommand().isScheduled() && !autoMessagePrinted) {
            Telemetry.print(
                    String.format(
                            "*** Auton %s in %.2f secs ***",
                            DriverStation.isAutonomousEnabled() ? "finished" : "CANCELLED",
                            Timer.getFPGATimestamp() - autonStart));
            autoMessagePrinted = true;
        }
    }

    /** Do nothing. */
    public Command doNothing() {
        return Commands.print("Do Nothing Auto ran").withName("Do Nothing");
    }

    /**
     * A routine step that sets a super state and moves straight on.
     *
     * @param state the super state to request
     * @return the step
     */
    public Command state(WantedSuperState state) {
        return robotSuperStructure.setStateCommand(state);
    }

    /**
     * A routine step that holds a super state for a fixed time, then returns to {@code IDLE}
     * ({@code IDLE} resolves to {@code AUTON_IDLE} in auto).
     *
     * @param state the super state to hold
     * @param seconds how long to hold it
     * @return the step
     */
    public Command holdState(WantedSuperState state, double seconds) {
        return Commands.sequence(
                        state(state), Commands.waitSeconds(seconds), state(WantedSuperState.IDLE))
                .withName("Auton.hold " + state);
    }

    /** A routine step that launches between path segments, then idles. */
    public Command launch() {
        return holdState(WantedSuperState.LAUNCH_WITH_SQUEEZE, LAUNCH_SECONDS)
                .withName("Auton.launch");
    }

    // ---- Routines ----

    // Named TWOMANOSTBTB because Java identifiers can't start with a digit; the auto file it loads
    // is "2MANOSTBTB FULL.auto".
    public Command TWOMANOSTBTB(boolean mirrored) {
        return routine(
                "2MANOSTBTB FULL",
                mirrored,
                Commands.waitSeconds(2),
                SpectrumAuton("2MANOSTBTB FULL", mirrored));
    }

    public Command OSCENT(boolean mirrored) {
        return routine(
                "OSCENT FULL",
                mirrored,
                SpectrumAuton("OSCENT FULL", mirrored),
                // Keeps launching after the path ends, at a standstill.
                state(WantedSuperState.AUTON_LAUNCH_WITH_SQUEEZE));
    }

    // ---- Building blocks ----

    /** A routine that is just one {@code .auto} file. */
    private Command single(String autoName, boolean mirrored) {
        return routine(autoName, mirrored, SpectrumAuton(autoName, mirrored));
    }

    /**
     * Sequences a routine's steps: path segments from {@link #SpectrumAuton}, state steps from
     * {@link #state}, {@link #holdState} or {@link #launch}, waits, or any other command. For
     * example:
     *
     * <pre>{@code
     * routine("TBTB Full", mirrored,
     *         SpectrumAuton("TBTB 1", mirrored), launch(),
     *         SpectrumAuton("TBTB 2", mirrored), launch(),
     *         SpectrumAuton("TBTB 3", mirrored));
     * }</pre>
     *
     * <p>It is named {@code "<fullAutoName> - Left"} or {@code " - Right"}, spaces included.
     * Robot.disabledPeriodic strips that suffix to find {@code <fullAutoName>.auto}, and uses it to
     * preview the paths and place the robot on the start pose, so for a routine built from segments
     * that file must be the whole routine end to end. The visualizer reads the suffix to decide
     * whether to mirror the poses.
     *
     * @param fullAutoName the {@code .auto} file describing the whole routine
     * @param mirrored whether the routine is mirrored
     * @param steps the routine's commands, in order
     * @return the routine command
     */
    private static Command routine(String fullAutoName, boolean mirrored, Command... steps) {
        return Commands.sequence(steps)
                .withName(fullAutoName + " - " + (mirrored ? "Right" : "Left"));
    }

    /**
     * Creates the PathPlannerAuto for one {@code .auto} file, built here at boot so its
     * trajectories are generated and cached before the match (PathPlanner's FollowPathCommand
     * generates the ideal trajectory in its constructor and reuses it at start if the robot is
     * still and within 30 deg of the path's starting heading).
     *
     * <p>Until 2026-09-16 this prepended {@code waitSeconds(0.01)}, a leftover from the 2025
     * migration. A wait command finishes on the scheduler loop after the one it started in, so it
     * cost a full loop, 25 to 40 ms at our loop period, of the robot standing still -- once per
     * segment in a multi-segment routine.
     *
     * @param autoName the name of the {@code .auto} file, without the extension
     * @param mirrored whether the autonomous routine should be mirrored
     * @return the auto command
     */
    public Command SpectrumAuton(String autoName, boolean mirrored) {
        verifyAutoFile(autoName);
        return new PathPlannerAuto(autoName, mirrored).withName(autoName);
    }

    /** Auto names the chooser was built with that have no {@code .auto} file on this rio. */
    private static final Set<String> missingAutoFiles = new LinkedHashSet<>();

    private static final Alert missingAutoFileAlert = new Alert("", AlertType.kError);

    /**
     * Checks at boot that every {@code .auto} file a routine uses exists in
     * deploy/pathplanner/autos.
     *
     * <p>PathPlanner reports a missing file to the Driver Station once at construction and then
     * runs an empty command, which is how Chezy 2026-09-19 QM4 sat still for auto: the code said
     * "OSCENT Full", the file said "OSCENT FULL.auto", and the rio's filesystem cares about the
     * difference while the Windows sim does not. Robot.logAutoSelection only checks the selected
     * routine's full file, so a misnamed segment of a multi-segment routine is caught here.
     *
     * @param autoName the exact file name without {@code .auto}
     */
    private static void verifyAutoFile(String autoName) {
        if (AutoBuilder.getAllAutoNames().contains(autoName)) {
            return;
        }
        missingAutoFiles.add(autoName);
        missingAutoFileAlert.setText(
                "No .auto file on the rio for: "
                        + String.join(", ", missingAutoFiles)
                        + " (names are case-sensitive on the rio). Those autos will do nothing.");
        missingAutoFileAlert.set(true);
        Telemetry.print(
                "!!! No .auto file named '"
                        + autoName
                        + "' in deploy/pathplanner/autos. That auto will do nothing.",
                PrintPriority.HIGH);
    }

    /**
     * Retrieves the autonomous command selected on the shuffleboard. Never null: the chooser has a
     * default option.
     *
     * @return the selected autonomous command
     */
    public Command getAutonomousCommand() {
        return pathChooser.getSelected();
    }
}
