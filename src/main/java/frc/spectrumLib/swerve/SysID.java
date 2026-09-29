package frc.spectrumLib.swerve;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;

/**
 * The three SysId characterization routines for a CTRE swerve drivetrain: translation, rotation and
 * steer gains.
 *
 * <p>Build one per robot with its {@link Swerve} subsystem, then bind {@link #sysIdQuasistatic} and
 * {@link #sysIdDynamic} to test-mode triggers. Which routine those run is decided by editing {@link
 * #RoutineToApply}.
 */
public class SysID {

    /** SysId routine that characterizes linear translation drive gains. */
    @Getter private final SysIdRoutine SysIdRoutineTranslation;

    /** SysId routine that characterizes rotational drive gains. */
    @Getter private final SysIdRoutine SysIdRoutineRotation;

    /** SysId routine that characterizes steer-module gains. */
    @Getter private final SysIdRoutine SysIdRoutineSteer;

    /** The routine currently selected for quasistatic and dynamic test commands. */
    private final SysIdRoutine RoutineToApply;

    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization =
            new SwerveRequest.SysIdSwerveTranslation();

    private final SwerveRequest.SysIdSwerveRotation RotationCharacterization =
            new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization =
            new SwerveRequest.SysIdSwerveSteerGains();

    /**
     * Builds all three routines and selects {@link #SysIdRoutineTranslation}. Change {@link
     * #RoutineToApply} at the bottom of this constructor to characterize a different one.
     */
    public SysID(Swerve swerve) {
        String stateTxt = "state";
        SysIdRoutineTranslation =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                Volts.of(4),
                                null,
                                state -> SignalLogger.writeString(stateTxt, state.toString())),
                        new SysIdRoutine.Mechanism(
                                volts ->
                                        swerve.setControl(
                                                TranslationCharacterization.withVolts(volts)),
                                null,
                                swerve));

        SysIdRoutineRotation =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                Volts.of(4),
                                null,
                                state -> SignalLogger.writeString(stateTxt, state.toString())),
                        new SysIdRoutine.Mechanism(
                                roationalRate ->
                                        swerve.setControl(
                                                RotationCharacterization.withRotationalRate(
                                                        roationalRate.baseUnitMagnitude())),
                                null,
                                swerve));

        SysIdRoutineSteer =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                Volts.of(7),
                                null,
                                state -> SignalLogger.writeString(stateTxt, state.toString())),
                        new SysIdRoutine.Mechanism(
                                volts -> swerve.setControl(SteerCharacterization.withVolts(volts)),
                                null,
                                swerve));

        RoutineToApply = SysIdRoutineTranslation;
    }

    /** A slow ramp of the active routine's output, in the given direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return RoutineToApply.quasistatic(direction);
    }

    /** A voltage step of the active routine, in the given direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return RoutineToApply.dynamic(direction);
    }
}
