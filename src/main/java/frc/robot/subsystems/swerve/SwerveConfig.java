package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.spectrumLib.hardware.Rio;
import lombok.Getter;

public class SwerveConfig {

    @Getter private final double simLoopPeriod = 0.005; // 5 ms

    @Getter private double driveGearRatio = 7.03;
    @Getter private double steerGearRatio = 26.09;

    // Estimated first, then fitted by hand until odometry matched the measured record.
    @Getter private Distance wheelRadius = Inches.of(1.978);

    /** Theoretical translational free speed at 12 V applied output. */
    @Getter private final LinearVelocity linearSpeedAt12Volts = MetersPerSecond.of(4.5);

    /** Theoretical rotational free speed at 12 V applied output. */
    @Getter private final AngularVelocity angularSpeedAt12Volts = DegreesPerSecond.of(540.00);

    @Getter
    private Slot0Configs steerGains =
            new Slot0Configs()
                    .withKP(500)
                    .withKI(0)
                    .withKD(20)
                    .withKS(0.15)
                    .withKV(1.0)
                    .withKA(0)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    @Getter
    private Slot0Configs driveGains =
            new Slot0Configs().withKP(10.0).withKI(0.0).withKD(0.0).withKS(4).withKV(0.0);

    @Getter
    private ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

    @Getter
    private ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

    /** The stator current at which the wheels start to slip. */
    @Getter private final Current slipCurrent = Amps.of(80);

    // Initial configs for the drive and steer motors and the CANcoder. These cannot be null, and
    // some are overwritten; check the with*InitialConfigs() API documentation.
    @Getter
    private TalonFXConfiguration driveInitialConfigs =
            new TalonFXConfiguration()
                    .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                    .withStatorCurrentLimit(slipCurrent)
                                    .withStatorCurrentLimitEnable(true)
                                    .withSupplyCurrentLimit(Amps.of(40.0))
                                    .withSupplyCurrentLimitEnable(true)
                                    .withSupplyCurrentLowerLimit(Amps.of(40.0))
                                    .withSupplyCurrentLowerTime(Seconds.of(1.0)));

    // Swerve azimuth does not need much torque, so a relatively low stator limit avoids brownouts
    // without hurting performance.
    @Getter
    private TalonFXConfiguration steerInitialConfigs =
            new TalonFXConfiguration()
                    .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                    .withStatorCurrentLimit(Amps.of(60.0))
                                    .withStatorCurrentLimitEnable(true)
                                    .withSupplyCurrentLimit(Amps.of(40.0))
                                    .withSupplyCurrentLimitEnable(true)
                                    .withSupplyCurrentLowerLimit(Amps.of(40.0))
                                    .withSupplyCurrentLowerTime(Seconds.of(1.0)));

    @Getter
    private final CANcoderConfiguration canCoderInitialConfigs = new CANcoderConfiguration();

    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    @Getter private final Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration();

    // Every rotation of the azimuth results in coupleRatio drive motor turns. MK5n first drive
    // stage
    // only, 54T over a 12T pinion.
    @Getter private final double coupleRatio = 54.0 / 12.0;

    @Getter private final boolean steerMotorReversed = false;
    // Drive motor inversion per side, the opposite of the CTRE template defaults: the bevel gears
    // on this drivetrain face the other way. Bench logs showed the robot moving opposite to every
    // command and to its own odometry, with wheel-derived rotation disagreeing in sign with the
    // gyro
    // and wheel-derived translation disagreeing in sign with camera-observed motion.
    @Getter private final boolean invertLeftSide = true;
    @Getter private final boolean invertRightSide = false;

    @Getter private final CANBus canBus = new CANBus(Rio.CANIVORE, "./logs/spectrum.hoot");
    @Getter private final int pigeonId = 0;

    // Only used for simulation.
    @Getter private final double steerInertia = 0.01;
    @Getter private final double driveInertia = 0.01;
    /** Simulated voltage needed to overcome friction. */
    @Getter private final Voltage steerFrictionVoltage = Volts.of(0.25);

    @Getter private final Voltage driveFrictionVoltage = Volts.of(0.25);

    @Getter private SwerveDrivetrainConstants drivetrainConstants;

    // Front Left
    @Getter private final int frontLeftDriveMotorId = 1;
    @Getter private final int frontLeftSteerMotorId = 2;
    @Getter private final int frontLeftEncoderId = 3;
    @Getter private Angle frontLeftEncoderOffset = Rotations.of(-0.83544921875);
    @Getter private final boolean frontLeftSteerInverted = false;

    @Getter private final Distance frontLeftXPos = Inches.of(10.25);
    @Getter private final Distance frontLeftYPos = Inches.of(13.375);

    // Front Right
    @Getter private final int frontRightDriveMotorId = 11;
    @Getter private final int frontRightSteerMotorId = 12;
    @Getter private final int frontRightEncoderId = 13;
    @Getter private Angle frontRightEncoderOffset = Rotations.of(-0.15234375);
    @Getter private final boolean frontRightSteerInverted = false;

    @Getter private final Distance frontRightXPos = Inches.of(10.25);
    @Getter private final Distance frontRightYPos = Inches.of(-13.375);

    // Back Left
    @Getter private final int backLeftDriveMotorId = 21;
    @Getter private final int backLeftSteerMotorId = 22;
    @Getter private final int backLeftEncoderId = 23;
    @Getter private Angle backLeftEncoderOffset = Rotations.of(-0.4794921875);
    @Getter private final boolean backLeftSteerInverted = false;

    @Getter private final Distance backLeftXPos = Inches.of(-7.368);
    @Getter private final Distance backLeftYPos = Inches.of(13.25);

    // Back Right
    @Getter private final int backRightDriveMotorId = 31;
    @Getter private final int backRightSteerMotorId = 32;
    @Getter private final int backRightEncoderId = 33;
    @Getter private Angle backRightEncoderOffset = Rotations.of(-0.84130859375);
    @Getter private final boolean backRightSteerInverted = false;

    @Getter private final Distance backRightXPos = Inches.of(-7.368);
    @Getter private final Distance backRightYPos = Inches.of(-13.25);

    @Getter
    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            frontLeft;

    @Getter
    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            frontRight;

    @Getter
    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            backLeft;

    @Getter
    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            backRight;

    @SuppressWarnings("unchecked")
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            [] getModules() {
        return new SwerveModuleConstants[] {frontLeft, frontRight, backLeft, backRight};
    }

    public SwerveConfig() {
        updateConfig();
    }

    public SwerveConfig updateConfig() {
        drivetrainConstants =
                new SwerveDrivetrainConstants()
                        .withCANBusName(canBus.getName())
                        .withPigeon2Id(pigeonId)
                        .withPigeon2Configs(pigeonConfigs);

        var constantCreator =
                new SwerveModuleConstantsFactory<
                                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                        .withDriveMotorGearRatio(driveGearRatio)
                        .withSteerMotorGearRatio(steerGearRatio)
                        .withWheelRadius(wheelRadius)
                        .withSlipCurrent(slipCurrent)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                        .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                        .withSpeedAt12Volts(linearSpeedAt12Volts)
                        .withSteerInertia(steerInertia)
                        .withDriveInertia(driveInertia)
                        .withSteerFrictionVoltage(steerFrictionVoltage)
                        .withDriveFrictionVoltage(driveFrictionVoltage)
                        .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                        .withCouplingGearRatio(coupleRatio)
                        .withDriveMotorInitialConfigs(driveInitialConfigs)
                        .withSteerMotorInitialConfigs(steerInitialConfigs)
                        .withEncoderInitialConfigs(canCoderInitialConfigs);

        frontLeft =
                constantCreator.createModuleConstants(
                        frontLeftSteerMotorId,
                        frontLeftDriveMotorId,
                        frontLeftEncoderId,
                        frontLeftEncoderOffset,
                        frontLeftXPos,
                        frontLeftYPos,
                        invertLeftSide,
                        steerMotorReversed,
                        frontLeftSteerInverted);

        frontRight =
                constantCreator.createModuleConstants(
                        frontRightSteerMotorId,
                        frontRightDriveMotorId,
                        frontRightEncoderId,
                        frontRightEncoderOffset,
                        frontRightXPos,
                        frontRightYPos,
                        invertRightSide,
                        steerMotorReversed,
                        frontRightSteerInverted);

        backLeft =
                constantCreator.createModuleConstants(
                        backLeftSteerMotorId,
                        backLeftDriveMotorId,
                        backLeftEncoderId,
                        backLeftEncoderOffset,
                        backLeftXPos,
                        backLeftYPos,
                        invertLeftSide,
                        steerMotorReversed,
                        backLeftSteerInverted);

        backRight =
                constantCreator.createModuleConstants(
                        backRightSteerMotorId,
                        backRightDriveMotorId,
                        backRightEncoderId,
                        backRightEncoderOffset,
                        backRightXPos,
                        backRightYPos,
                        invertRightSide,
                        steerMotorReversed,
                        backRightSteerInverted);

        return this;
    }

    /**
     * Sets the four CANcoder offsets, in rotations: front left, front right, back left, back right.
     */
    public SwerveConfig configEncoderOffsets(
            double frontLeft, double frontRight, double backLeft, double backRight) {
        frontLeftEncoderOffset = Rotations.of(frontLeft);
        frontRightEncoderOffset = Rotations.of(frontRight);
        backLeftEncoderOffset = Rotations.of(backLeft);
        backRightEncoderOffset = Rotations.of(backRight);
        return updateConfig();
    }
}
