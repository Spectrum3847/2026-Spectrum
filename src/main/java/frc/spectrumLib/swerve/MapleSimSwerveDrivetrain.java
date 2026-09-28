package frc.spectrumLib.swerve;

// Copyright 2021-2025 Iron Maple 5516
// Original Source:
// https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/templates/CTRE%20Swerve%20with%20maple-sim/src/main/java/frc/robot/utils/simulation/MapleSimSwerveDrivetrain.java
//
// This code is licensed under MIT license (see https://mit-license.org/)

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

/**
 * Injects Maple-Sim data into a CTRE {@link SwerveDrivetrain}. Stands in for {@link
 * com.ctre.phoenix6.swerve.SimSwerveDrivetrain}.
 */
public class MapleSimSwerveDrivetrain {
    /** Sim state of the Pigeon2, written by {@link #update()}. */
    private final Pigeon2SimState pigeonSim;

    private final SimSwerveModule[] simModules;

    public final SwerveDriveSimulation mapleSimDrive;

    /**
     * Builds the sim starting at pose zero, then takes over the global sim state: it overrides the
     * simulation timings and installs a fresh {@link Arena2026Rebuilt} with efficiency mode on as
     * the SimulatedArena instance.
     *
     * @param moduleLocations module locations in the order FL, FR, BL, BR
     * @param driveMotorModel drive motor model, typically DCMotor.getKrakenX60Foc()
     */
    @SuppressWarnings("unchecked")
    public MapleSimSwerveDrivetrain(
            Time simPeriod,
            Mass robotMassWithBumpers,
            Distance bumperLengthX,
            Distance bumperWidthY,
            DCMotor driveMotorModel,
            DCMotor steerMotorModel,
            double wheelCOF,
            Translation2d[] moduleLocations,
            Pigeon2 pigeon,
            SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                            ...
                    moduleConstants) {
        this.pigeonSim = pigeon.getSimState();
        simModules = new SimSwerveModule[moduleConstants.length];
        DriveTrainSimulationConfig simulationConfig =
                DriveTrainSimulationConfig.Default()
                        .withRobotMass(robotMassWithBumpers)
                        .withBumperSize(bumperLengthX, bumperWidthY)
                        .withGyro(COTS.ofPigeon2())
                        .withCustomModuleTranslations(moduleLocations)
                        .withSwerveModule(
                                new SwerveModuleSimulationConfig(
                                        driveMotorModel,
                                        steerMotorModel,
                                        moduleConstants[0].DriveMotorGearRatio,
                                        moduleConstants[0].SteerMotorGearRatio,
                                        Volts.of(moduleConstants[0].DriveFrictionVoltage),
                                        Volts.of(moduleConstants[0].SteerFrictionVoltage),
                                        Meters.of(moduleConstants[0].WheelRadius),
                                        KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
                                        wheelCOF));
        mapleSimDrive = new SwerveDriveSimulation(simulationConfig, Pose2d.kZero);

        SwerveModuleSimulation[] moduleSimulations = mapleSimDrive.getModules();
        for (int i = 0; i < this.simModules.length; i++)
            simModules[i] =
                    new SimSwerveModule(moduleConstants[0], moduleSimulations[i], modules[i]);

        Arena2026Rebuilt arena = new Arena2026Rebuilt(false);
        arena.setEfficiencyMode(true);

        SimulatedArena.overrideSimulationTimings(simPeriod, 1);
        SimulatedArena.overrideInstance(arena);
        SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
    }

    /** Steps the arena and copies the resulting yaw and yaw rate into the Pigeon2's sim state. */
    public void update() {
        SimulatedArena.getInstance().simulationPeriodic();
        pigeonSim.setRawYaw(mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
        pigeonSim.setAngularVelocityZ(
                RadiansPerSecond.of(
                        mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative()
                                .omegaRadiansPerSecond));
    }

    /** Physics sim for one CTRE swerve module, wired to that module's motor controllers. */
    protected static class SimSwerveModule {
        public final SwerveModuleConstants<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                moduleConstant;

        public final SwerveModuleSimulation moduleSimulation;

        public SimSwerveModule(
                SwerveModuleConstants<
                                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                        moduleConstant,
                SwerveModuleSimulation moduleSimulation,
                SwerveModule<TalonFX, TalonFX, CANcoder> module) {
            this.moduleConstant = moduleConstant;
            this.moduleSimulation = moduleSimulation;
            moduleSimulation.useDriveMotorController(
                    new TalonFXMotorControllerSim(module.getDriveMotor()));
            moduleSimulation.useSteerMotorController(
                    new TalonFXMotorControllerWithRemoteCanCoderSim(
                            module.getSteerMotor(), module.getEncoder()));
        }
    }

    /**
     * Feeds Maple-Sim's rotor state into a TalonFX's sim state and returns the voltage the motor
     * asks for.
     */
    public static class TalonFXMotorControllerSim implements SimulatedMotorController {
        public final int id;

        private final TalonFXSimState talonFXSimState;

        public TalonFXMotorControllerSim(TalonFX talonFX) {
            this.id = talonFX.getDeviceID();
            this.talonFXSimState = talonFX.getSimState();
        }

        /**
         * Writes the rotor-side angles and the battery voltage, then returns the motor voltage the
         * controller is requesting. The mechanism-side arguments are ignored, since this path only
         * ever writes the rotor side.
         */
        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            talonFXSimState.setRawRotorPosition(encoderAngle);
            talonFXSimState.setRotorVelocity(encoderVelocity);
            talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

            return talonFXSimState.getMotorVoltageMeasure();
        }
    }

    /**
     * Steer adapter that also drives the remote CANcoder's sim state. Use it when the steer motor
     * takes feedback from a remote CANcoder rather than its own encoder.
     */
    @SuppressWarnings("all")
    public static class TalonFXMotorControllerWithRemoteCanCoderSim
            extends TalonFXMotorControllerSim {
        private final int encoderId;

        private final CANcoderSimState remoteCancoderSimState;

        public TalonFXMotorControllerWithRemoteCanCoderSim(TalonFX talonFX, CANcoder cancoder) {
            super(talonFX);
            this.remoteCancoderSimState = cancoder.getSimState();

            this.encoderId = cancoder.getDeviceID();
        }

        /**
         * Writes the mechanism-side state to the CANcoder, since that is the side a remote encoder
         * sits on, then lets the parent handle the rotor side and the return value.
         */
        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            remoteCancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            remoteCancoderSimState.setRawPosition(mechanismAngle);
            remoteCancoderSimState.setVelocity(mechanismVelocity);

            return super.updateControlSignal(
                    mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
        }
    }

    /**
     * Applies simulation-only constant overrides to every module. See {@link
     * #regulateModuleConstantForSimulation(SwerveModuleConstants)} for what changes and why.
     */
    public static SwerveModuleConstants<?, ?, ?>[] regulateModuleConstantsForSimulation(
            SwerveModuleConstants<?, ?, ?>[] moduleConstants) {
        for (SwerveModuleConstants<?, ?, ?> moduleConstant : moduleConstants)
            regulateModuleConstantForSimulation(moduleConstant);

        return moduleConstants;
    }

    /**
     * Overwrites a module's constants for simulation: clears the drive, steer, and encoder
     * inversions plus the encoder offset, then substitutes sim-tuned gains, gear ratio, friction
     * voltages, and steer inertia. The sim needs this because inverted drive configs upset the
     * drive PID and a non-zero CanCoder offset upsets module state optimization. Leaves the
     * constants alone on a real robot.
     */
    private static void regulateModuleConstantForSimulation(
            SwerveModuleConstants<?, ?, ?> moduleConstants) {
        if (RobotBase.isReal()) return;

        moduleConstants
                .withEncoderOffset(0)
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(false)
                .withEncoderInverted(false)
                .withSteerMotorGains(
                        new Slot0Configs()
                                .withKP(1000.0)
                                .withKI(0)
                                .withKD(60.0)
                                .withKS(0.15)
                                .withKV(1.5)
                                .withKA(0)
                                .withStaticFeedforwardSign(
                                        StaticFeedforwardSignValue.UseClosedLoopSign))
                .withSteerMotorGearRatio(21.428571428571427)
                .withDriveFrictionVoltage(Volts.of(0.1))
                .withSteerFrictionVoltage(Volts.of(0.05))
                .withSteerInertia(KilogramSquareMeters.of(0.05));
    }
}
