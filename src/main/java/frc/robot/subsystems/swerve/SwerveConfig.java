package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.spectrumLib.hardware.Rio;
import lombok.Getter;
import lombok.Setter;

public class SwerveConfig {

    @Getter private final double simLoopPeriod = 0.005; // 5 ms

    @Getter @Setter private double deadband = 0.05; // 5% joystick deadband
    @Getter @Setter private double aimDeadband = 0.01; // 1% deadband for aiming modes

    @Getter @Setter private double driveGearRatio = 6.03;
    @Getter @Setter private double steerGearRatio = 26.09;

    // Fudged from the physical radius so odometry matches a measured run
    @Getter @Setter private Distance wheelRadius = Inches.of(1.964); // 0.0499 m

    // Theoretical free speed at 12 V
    @Getter @Setter private LinearVelocity linearSpeedAt12Volts = MetersPerSecond.of(5.12);

    // Theoretical rotational free speed at 12 V
    @Getter @Setter private AngularVelocity angularSpeedAt12Volts = DegreesPerSecond.of(540.00);

    @Getter private double kPRotationController = 5.0;
    @Getter private double kIRotationController = 0.0;
    @Getter private double kDRotationController = 0.0;

    // Blue alliance forward is 0 degrees, toward the red wall
    @Getter private final Rotation2d blueAlliancePerspectiveRotation = Rotation2d.kZero;
    // Red alliance forward is 180 degrees, toward the blue wall
    @Getter private final Rotation2d redAlliancePerspectiveRotation = Rotation2d.k180deg;

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
            new Slot0Configs().withKP(10.0).withKI(0.0).withKD(0.0).withKS(4.0).withKV(0.0);

    @Getter
    private ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

    @Getter
    private ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

    // Stator current at which the drive wheels start to slip
    @Getter @Setter private Current slipCurrent = Amps.of(80);

    // Cannot be null. Check the with*InitialConfigs() API before changing these.
    @Getter
    private TalonFXConfiguration driveInitialConfigs =
            new TalonFXConfiguration()
                    .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                    .withStatorCurrentLimit(Amps.of(80.0))
                                    .withStatorCurrentLimitEnable(true)
                                    .withSupplyCurrentLimit(Amps.of(30.0))
                                    .withSupplyCurrentLimitEnable(true)
                                    .withSupplyCurrentLowerLimit(Amps.of(30.0)));

    // Steer needs little torque, so a low limit avoids brownouts without hurting performance
    @Getter
    private TalonFXConfiguration steerInitialConfigs =
            new TalonFXConfiguration()
                    .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                    .withStatorCurrentLimit(Amps.of(60))
                                    .withStatorCurrentLimitEnable(true));

    @Getter private CANcoderConfiguration canCoderInitialConfigs = new CANcoderConfiguration();

    // Set to null to skip applying Pigeon 2 configs
    @Getter private Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration();

    // Drive motor turns per azimuth rotation
    @Getter private double coupleRatio = 4.5;

    @Getter @Setter private boolean steerMotorReversed = false;
    @Getter @Setter private boolean invertLeftSide = false;
    @Getter @Setter private boolean invertRightSide = true;

    @Getter @Setter private CANBus canBus = new CANBus(Rio.CANIVORE, "./logs/spectrum.hoot");
    @Getter private int pigeonId = 0;

    // These are only used for simulation
    @Getter private double steerInertia = 0.01;
    @Getter private double driveInertia = 0.01;
    // Simulated voltage necessary to overcome friction
    @Getter private Voltage steerFrictionVoltage = Volts.of(0.25);
    @Getter private Voltage driveFrictionVoltage = Volts.of(0.25);

    @Getter private SwerveDrivetrainConstants drivetrainConstants;

    @Getter
    private SwerveModuleConstantsFactory<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            constantCreator;

    private final double wheelBaseInches = 21.75;
    private final double trackWidthInches = 21.75;

    // Center to module distance. MapleSim calls it a radius, but it is the diagonal.
    @Getter
    private final double drivebaseRadiusInches =
            Math.hypot(wheelBaseInches / 2.0, trackWidthInches / 2.0);

    @Getter
    private final double drivebaseRadiusMeters = Units.inchesToMeters(drivebaseRadiusInches);

    @Getter private int frontLeftDriveMotorId = 1;
    @Getter private int frontLeftSteerMotorId = 2;
    @Getter private int frontLeftEncoderId = 3;
    @Getter private Angle frontLeftEncoderOffset = Rotations.of(-0.83544921875);
    @Getter private boolean frontLeftSteerInverted = false;

    @Getter private Distance frontLeftXPos = Inches.of(wheelBaseInches / 2);
    @Getter private Distance frontLeftYPos = Inches.of(trackWidthInches / 2);

    @Getter private int frontRightDriveMotorId = 11;
    @Getter private int frontRightSteerMotorId = 12;
    @Getter private int frontRightEncoderId = 13;
    @Getter private Angle frontRightEncoderOffset = Rotations.of(-0.15234375);
    @Getter private boolean frontRightSteerInverted = false;

    @Getter private Distance frontRightXPos = Inches.of(wheelBaseInches / 2);
    @Getter private Distance frontRightYPos = Inches.of(-trackWidthInches / 2);

    @Getter private int backLeftDriveMotorId = 21;
    @Getter private int backLeftSteerMotorId = 22;
    @Getter private int backLeftEncoderId = 23;
    @Getter private Angle backLeftEncoderOffset = Rotations.of(-0.4794921875);
    @Getter private boolean backLeftSteerInverted = false;

    @Getter private Distance backLeftXPos = Inches.of(-wheelBaseInches / 2);
    @Getter private Distance backLeftYPos = Inches.of(trackWidthInches / 2);

    @Getter private int backRightDriveMotorId = 31;
    @Getter private int backRightSteerMotorId = 32;
    @Getter private int backRightEncoderId = 33;
    @Getter private Angle backRightEncoderOffset = Rotations.of(-0.84130859375);
    @Getter private boolean backRightSteerInverted = false;

    @Getter private Distance backRightXPos = Inches.of(-wheelBaseInches / 2);
    @Getter private Distance backRightYPos = Inches.of(-trackWidthInches / 2);

    @Getter @Setter private double targetHeading = 0;

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

    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                    []
            modules;

    @SuppressWarnings("unchecked")
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            [] getModules() {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            modules = new SwerveModuleConstants[] {frontLeft, frontRight, backLeft, backRight};
        } else {
            throw new IllegalStateException("One or more SwerveModuleConstants are null");
        }
        return modules;
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

        constantCreator =
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

    public SwerveConfig configEncoderOffsets(
            double frontLeft, double frontRight, double backLeft, double backRight) {
        frontLeftEncoderOffset = Rotations.of(frontLeft);
        frontRightEncoderOffset = Rotations.of(frontRight);
        backLeftEncoderOffset = Rotations.of(backLeft);
        backRightEncoderOffset = Rotations.of(backRight);
        return updateConfig();
    }
}
