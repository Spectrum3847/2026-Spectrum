package frc.robot.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.rebuilt.Field;
import frc.rebuilt.Zones;
import frc.robot.Robot;
import frc.robot.pilot.Pilot;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class SwerveStates {
    static Swerve swerve = Robot.getSwerve();
    static SwerveConfig config = Robot.getConfig().swerve;
    static Pilot pilot = Robot.getPilot();
    static Zones zones = new Zones();
    static Field field = new Field();

    /* Swerve Request Types */
    private static final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDeadband(
                    config.getSpeedAt12Volts().in(MetersPerSecond) * config.getDeadband())
            .withRotationalDeadband(config.getMaxAngularRate() * config.getDeadband())
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private static final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(
                    config.getSpeedAt12Volts().in(MetersPerSecond) * config.getDeadband())
            .withRotationalDeadband(config.getMaxAngularRate() * config.getDeadband())
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final SwerveRequest.SwerveDriveBrake swerveXBreak = new SwerveRequest.SwerveDriveBrake();

    static Command pilotSteerCommand =
            log(pilotDrive().withName("SwerveCommands.pilotSteer").ignoringDisable(true));

    protected static void setupDefaultCommand() {
        swerve.setDefaultCommand(pilotSteerCommand);
    }

    protected static void setStates() {
        // Force back to manual steering when we steer
        pilot.steer.whileTrue(
                swerve.getDefaultCommand()); 

        pilot.fpv_LS.whileTrue(log(fpvDrive()));
        pilot.AButton.whileTrue(log(snakeDrive()));

        pilot.upReorient.onTrue(log(reorientForward()));
        pilot.leftReorient.onTrue(log(reorientLeft()));
        pilot.downReorient.onTrue(log(reorientBack()));
        pilot.rightReorient.onTrue(log(reorientRight()));
    }

    /* ------------------------- Pilot Commands --------------------------------------- */

    /** Drive the robot using left stick and control orientation using the right stick. */
    protected static Command pilotDrive() {
        return drive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::getDriveCCWPositive)
                .withName("Swerve.PilotDrive");
    }

    /** Drive the robot with its front bumper as the forward direction. */
    protected static Command fpvDrive() {
        return fpvDrive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::getDriveCCWPositive)
                .withName("Swerve.PilotFPVDrive");
    }

    /** Align the robot to the given x, y, and heading goals. */
    public static Command alignDrive(
            DoubleSupplier xGoalMeters, DoubleSupplier yGoalMeters, DoubleSupplier headingRadians) {
        if (Field.isRed()) {
            return resetXController()
                    .andThen(
                            resetYController(),
                            resetTurnController(),
                            drive(
                                    () -> -getAlignToX(xGoalMeters).getAsDouble(),
                                    () -> -getAlignToY(yGoalMeters).getAsDouble(),
                                    () -> getAlignHeading(headingRadians, true).getAsDouble()));
        }

        return resetXController()
                .andThen(
                        resetYController(),
                        resetTurnController(),
                        drive(
                                getAlignToX(xGoalMeters),
                                getAlignToY(yGoalMeters),
                                getAlignHeading(headingRadians, true)));
    }

    /** Drive the robot with the robot's orientation snapping to the closest cardinal direction. */
    protected static Command snapSteerDrive() {
        return drive(
                pilot::getDriveFwdPositive,
                pilot::getDriveLeftPositive,
                pilot::chooseCardinalDirections)
                .withName("Swerve.PilotStickSteer");
    }

    /** Drive the robot with the front bumper trying to match the robot's motion. */
    protected static Command snakeDrive() {
        return aimDrive(
                pilot::getDriveFwdPositive,
                pilot::getDriveLeftPositive,
                pilot::getPilotStickAngle)
                .withName("Swerve.SnakeDrive");
    }

    /** Turn the swerve wheels to an X to prevent the robot from moving */
    protected static Command xBrake() {
        return swerve.applyRequest(() -> swerveXBreak).withName("Swerve.Xbrake");
    }

    protected static Command pilotAimDrive(DoubleSupplier targetDegrees) {
            return aimDrive(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive, targetDegrees)
                    .withName("Swerve.PilotAimDrive");
    }

    private static DoubleSupplier getAlignToX(DoubleSupplier xGoalMeters) {
        return swerve.calculateXController(xGoalMeters);
    }

    private static DoubleSupplier getAlignToY(DoubleSupplier yGoalMeters) {
        return swerve.calculateYController(yGoalMeters);
    }

    private static DoubleSupplier getAlignHeading(DoubleSupplier headingRadians, boolean usehold) {
        return () -> swerve.calculateRotationController(headingRadians, usehold);
    }

    protected static Command headingLockDrive() {
        return headingLock(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive)
                .withName("Swerve.PilotHeadingLockDrive");
    }

    protected static Command lockToClosest45degDrive() {
        return lockToClosest45deg(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive)
                .withName("Swerve.PilotLockTo45degDrive");
    }

    protected static Command lockToClosestFieldAngleDrive() {
        return lockToClosestFieldAngle(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive)
                .withName("Swerve.PilotLockToFieldAngleDrive");
    }

     // **********************   Helper Commands    ****************************

    protected static Command resetXController() {
        return swerve.runOnce(() -> swerve.resetXController()).withName("ResetXController");
    }

    protected static Command resetYController() {
        return swerve.runOnce(() -> swerve.resetYController()).withName("ResetYController");
    }

    protected static Command resetTurnController() {
        return swerve.runOnce(() -> swerve.resetRotationController())
                .withName("ResetTurnController");
    }

    protected static Command setTargetHeading(DoubleSupplier targetHeading) {
        return Commands.runOnce(() -> config.setTargetHeading(targetHeading.getAsDouble()))
                .withName("SetTargetHeading");
    }

    // Uses m/s and rad/s
    private static Command drive(
            DoubleSupplier fwdPositive, DoubleSupplier leftPositive, DoubleSupplier ccwPositive) {
        return swerve.applyRequest(
                        () ->
                                fieldCentricDrive
                                        .withVelocityX(fwdPositive.getAsDouble())
                                        .withVelocityY(leftPositive.getAsDouble())
                                        .withRotationalRate(ccwPositive.getAsDouble()))
                .withName("Swerve.drive");
    }

    private static Command fpvDrive(
            DoubleSupplier fwdPositive, DoubleSupplier leftPositive, DoubleSupplier ccwPositive) {
        return swerve.applyRequest(
                        () ->
                                robotCentric
                                        .withVelocityX(fwdPositive.getAsDouble())
                                        .withVelocityY(leftPositive.getAsDouble())
                                        .withRotationalRate(ccwPositive.getAsDouble()))
                .withName("Swerve.fpvDrive");
    }

    protected static Command fpvAimDrive(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier targetRadians) {
        return resetTurnController()
                .andThen(fpvDrive(velocityX, velocityY, getAlignHeading(targetRadians, true)))
                .withName("Swerve.fpvAimDrive");
    }

    /**
     * Reset the turn controller and then run the drive command with a angle supplier. This can be
     * used for aiming at a goal or heading locking, etc.
     */
    protected static Command aimDrive(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier targetRadians) {
        return resetTurnController()
                .andThen(drive(velocityX, velocityY, getAlignHeading(targetRadians, false)))
                .withName("Swerve.aimDrive");
    }

    /**
     * Reset the turn controller, set the target heading to the current heading(end that command
     * immediately), and then run the drive command with the Rotation controller. The rotation
     * controller will only engage if you are driving x or y.
     */
    protected static Command headingLock(DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return resetTurnController()
                .andThen(
                        setTargetHeading(() -> swerve.getRotation().getRadians()),
                        drive(
                                velocityX,
                                velocityY,
                                rotateToHeadingWhenMoving(
                                        velocityX, velocityY, () -> config.getTargetHeading())))
                .withName("Swerve.HeadingLock");
    }

    protected static Command lockToClosest45deg(
            DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return resetTurnController()
                .andThen(
                        setTargetHeading(() -> swerve.getClosest45()),
                        drive(
                                velocityX,
                                velocityY,
                                rotateToHeadingWhenMoving(
                                        velocityX, velocityY, () -> swerve.getClosest45())))
                .withName("Swerve.LockTo45deg");
    }

    protected static Command lockToClosestFieldAngle(
            DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return resetTurnController()
                .andThen(
                        setTargetHeading(() -> swerve.getClosestFieldAngle()),
                        drive(
                                velocityX,
                                velocityY,
                                rotateToHeadingWhenMoving(
                                        velocityX, velocityY, () -> swerve.getClosestFieldAngle())))
                .withName("Swerve.LockToFieldAngle");
    }

    private static DoubleSupplier rotateToHeadingWhenMoving(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier heading) {
        return () -> {
            if (Math.abs(velocityX.getAsDouble()) < 0.5
                    && Math.abs(velocityY.getAsDouble()) < 0.5) {
                return 0;
            } else {
                return getAlignHeading(heading, true).getAsDouble();
            }
        };
    }

    /**
     * *******************************************************************************************
     * Reorient Commands
     */
    protected static Command reorientForward() {
        return swerve.reorientPilotAngle(0).withName("Swerve.reorientForward");
    }

    protected static Command reorientLeft() {
        return swerve.reorientPilotAngle(90).withName("Swerve.reorientLeft");
    }

    protected static Command reorientBack() {
        return swerve.reorientPilotAngle(180).withName("Swerve.reorientBack");
    }

    protected static Command reorientRight() {
        return swerve.reorientPilotAngle(270).withName("Swerve.reorientRight");
    }

    protected static Command cardinalReorient() {
        return swerve.cardinalReorient().withName("Swerve.cardinalReorient");
    }

    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
