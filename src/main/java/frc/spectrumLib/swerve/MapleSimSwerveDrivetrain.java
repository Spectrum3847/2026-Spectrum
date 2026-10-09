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
 * Feeds Maple-Sim results into a CTRE {@link com.ctre.phoenix6.swerve.SwerveDrivetrain}, in place
 * of {@link com.ctre.phoenix6.swerve.SimSwerveDrivetrain}.
 */
public class MapleSimSwerveDrivetrain {
    /** Simulation state of the Pigeon2 gyro, used to inject computed yaw and angular velocity. */
    private final Pigeon2SimState pigeonSim;

    /** Simulated representations of each swerve module, indexed FL/FR/BL/BR. */
    private final SimSwerveModule[] simModules;

    public final SwerveDriveSimulation mapleSimDrive;

    /**
     * @param bumperLengthX the bumper length along the X axis, which sets the robot's collision
     *     space
     * @param bumperWidthY the bumper width along the Y axis, which sets the robot's collision space
     * @param driveMotorModel the drive motor model, typically {@code DCMotor.getKrakenX60Foc()}
     * @param steerMotorModel the steer motor model, typically {@code DCMotor.getKrakenX60Foc()}
     * @param moduleLocations the module positions, ordered FL, FR, BL, BR
     * @param modules the {@link SwerveModule}s, usually from {@link SwerveDrivetrain#getModules()}
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

    /**
     * Advances the arena one step, which drives the module motor sims, then writes the resulting
     * yaw and yaw rate into the {@link Pigeon2} sim state.
     */
    public void update() {
        SimulatedArena.getInstance().simulationPeriodic();
        pigeonSim.setRawYaw(mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
        pigeonSim.setAngularVelocityZ(
                RadiansPerSecond.of(
                        mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative()
                                .omegaRadiansPerSecond));
    }

    /** One module's Maple-Sim physics, wired to the CTRE motor controllers. */
    protected static class SimSwerveModule {
        /** Constants (gear ratios, friction voltages, wheel radius, etc.) for this module. */
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
     * Adapts a {@link TalonFX} for Maple-Sim's {@link SimulatedMotorController}, forwarding the
     * simulated rotor state into the CTRE sim state.
     */
    public static class TalonFXMotorControllerSim implements SimulatedMotorController {
        public final int id;

        /** Where this adapter injects position, velocity and voltage. */
        private final TalonFXSimState talonFXSimState;

        public TalonFXMotorControllerSim(TalonFX talonFX) {
            this.id = talonFX.getDeviceID();
            this.talonFXSimState = talonFX.getSimState();
        }

        /**
         * Writes the rotor-side encoder state and the simulated battery voltage into the TalonFX
         * sim state, then returns the voltage the controller's closed loop asks for. The
         * mechanism-side arguments are unused, since a TalonFX closes its loop on the rotor sensor.
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
     * Drives a remote CANcoder sim state as well, for a steer motor whose feedback device is a
     * remote CANcoder.
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
         * Writes the mechanism-side angle and velocity into the remote CANcoder sim state, then
         * hands all four values to the parent, which forwards the rotor-side pair to the TalonFX.
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
     * Applies the simulation-only adjustments to every module constant. Does nothing on real
     * hardware.
     */
    public static SwerveModuleConstants<?, ?, ?>[] regulateModuleConstantsForSimulation(
            SwerveModuleConstants<?, ?, ?>[] moduleConstants) {
        for (SwerveModuleConstants<?, ?, ?> moduleConstant : moduleConstants)
            regulateModuleConstantForSimulation(moduleConstant);

        return moduleConstants;
    }

    /**
     * Adjusts one module's constants for simulation, to work around simulation bugs rather than
     * robot behaviour. Motor inversions go to false because an inverted configuration upsets the
     * drive PID, and the CANcoder offset goes to zero because a non-zero one upsets the module
     * state optimization. The steer gains set below are placeholders that keep the sim stable, not
     * values fit to anything.
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
