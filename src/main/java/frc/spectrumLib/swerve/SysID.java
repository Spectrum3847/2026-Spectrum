package frc.spectrumLib.swerve;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;

/**
 * SysId routines for a CTRE swerve drivetrain: translation, rotation, and steer gains. Construct
 * one per robot and bind {@link #sysIdQuasistatic} and {@link #sysIdDynamic} in test mode.
 */
public class SysID {

    @Getter private final SysIdRoutine SysIdRoutineTranslation;

    @Getter private final SysIdRoutine SysIdRoutineRotation;

    @Getter private final SysIdRoutine SysIdRoutineSteer;

    /** Routine that {@link #sysIdQuasistatic} and {@link #sysIdDynamic} run. */
    private final SysIdRoutine RoutineToApply;

    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization =
            new SwerveRequest.SysIdSwerveTranslation();

    private final SwerveRequest.SysIdSwerveRotation RotationCharacterization =
            new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization =
            new SwerveRequest.SysIdSwerveSteerGains();

    /**
     * Builds all three routines. Edit the last line to pick which one runs. Steer runs at 7 V,
     * translation and rotation at 4 V.
     *
     * @param swerve the {@link Swerve} subsystem commanded during characterization
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

    /** Slow ramp command for the active routine. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return RoutineToApply.quasistatic(direction);
    }

    /** Step voltage command for the active routine. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return RoutineToApply.dynamic(direction);
    }
}
