package frc.spectrumLib.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;

/**
 * WPILib-backed simulation of a linear (elevator-style) mechanism driven by one or more Kraken X60
 * motors. Advances the TalonFX sim state on the shared {@link SimLoop} thread and animates a static
 * backing ligament plus a moving stage ligament in a {@link Mechanism2d} canvas. Implements {@link
 * Mount} so other mechanisms can attach to the moving stage, and {@link Mountable} so this stage
 * can itself attach to a parent mount.
 */
public class LinearSim implements Mount, Mountable {
    private ElevatorSim elevatorSim;

    private final MechanismRoot2d staticRoot;
    private final MechanismRoot2d root;
    private final MechanismLigament2d staticMech2d;
    private final MechanismLigament2d m_elevatorMech2d;
    @Getter private LinearConfig config;

    private TalonFXSimState linearMotorSim;

    /** Always {@link MountType#LINEAR}, so children apply linear placement. */
    @Getter private final MountType mountType = MountType.LINEAR;

    /**
     * @param mech the Mechanism2d canvas to draw the stage on
     * @param name prefix for this sim's Mechanism2d element labels
     */
    public LinearSim(LinearConfig config, Mechanism2d mech, TalonFX motor, String name) {
        this.config = config;
        this.linearMotorSim = SimMotor.simState(motor, config.isReversedLinkage());

        this.elevatorSim =
                new ElevatorSim(
                        DCMotor.getKrakenX60Foc(config.getNumMotors()),
                        config.getElevatorGearing(),
                        config.getCarriageMassKg(),
                        config.getDrumRadius(),
                        config.getMinHeight(),
                        config.getMaxHeight(),
                        true,
                        0);

        staticRoot =
                mech.getRoot(name + " 1StaticRoot", config.getInitialX(), config.getInitialY());
        staticMech2d =
                staticRoot.append(
                        new MechanismLigament2d(
                                name + " 1Static",
                                config.getStaticLength(),
                                config.getAngle(),
                                config.getLineWidth(),
                                new Color8Bit(Color.kOrange)));

        root = mech.getRoot(name + " Root", config.getInitialX(), config.getInitialY());
        m_elevatorMech2d =
                root.append(
                        new MechanismLigament2d(
                                name,
                                config.getMovingLength(),
                                config.getAngle(),
                                config.getLineWidth(),
                                new Color8Bit(Color.kBlack)));

        SimLoop.register(this::update);
    }

    private double getRotationPerSec() {
        return drumRotations(elevatorSim.getVelocityMetersPerSecond());
    }

    private double getRotations() {
        return drumRotations(elevatorSim.getPositionMeters());
    }

    /** Converts carriage travel (metres, or metres/second) to motor rotations (or rotations/s). */
    private double drumRotations(double meters) {
        return (meters / (2 * Math.PI * config.getDrumRadius())) * config.getElevatorGearing();
    }

    /** One sim tick: steps the elevator, feeds the rotor state back, and redraws. */
    public void update(double dt) {
        elevatorSim.setInput(linearMotorSim.getMotorVoltage());
        elevatorSim.update(dt);

        linearMotorSim.setRotorVelocity(getRotationPerSec());
        linearMotorSim.setRawRotorPosition(getRotations());

        double displacement = elevatorSim.getPositionMeters();

        double angle = stageAngleDegrees();
        if (config.isMounted()) {
            config.setStaticRootX(getUpdatedX(config));
            config.setStaticRootY(getUpdatedY(config));
            staticRoot.setPosition(config.getStaticRootX(), config.getStaticRootY());
            staticMech2d.setAngle(angle);
            m_elevatorMech2d.setAngle(angle);
        }
        // Only a mounted stage moves its static root, so unmounted this stays put.
        double radians = Math.toRadians(angle);
        root.setPosition(
                config.getStaticRootX() + displacement * Math.cos(radians),
                config.getStaticRootY() + displacement * Math.sin(radians));
    }

    /** The stage angle in degrees, following the parent mount when mounted. */
    private double stageAngleDegrees() {
        if (!config.isMounted()) {
            return config.getAngle();
        }
        Mount mount = config.getMount();
        return switch (mount.getMountType()) {
            case ARM -> config.getAngle() + Math.toDegrees(mount.getAngle());
            case LINEAR -> config.getAngle()
                    + Math.toDegrees(mount.getAngle() - config.getInitMountAngle());
        };
    }

    /**
     * Horizontal distance the carriage has travelled, projected along the stage angle.
     *
     * @return horizontal displacement in metres
     */
    public double getDisplacementX() {
        double angle = stageAngleDegrees();
        return elevatorSim.getPositionMeters() * Math.cos(Math.toRadians(angle))
                + (config.getStaticRootX() - config.getInitialX());
    }

    /**
     * Vertical distance the carriage has travelled, projected along the stage angle.
     *
     * @return vertical displacement in metres
     */
    public double getDisplacementY() {
        double angle = stageAngleDegrees();
        return elevatorSim.getPositionMeters() * Math.sin(Math.toRadians(angle))
                + (config.getStaticRootY() - config.getInitialY());
    }

    /**
     * Stage angle in radians, plus the parent mount's angle when mounted.
     *
     * @return effective stage angle in radians
     */
    public double getAngle() {
        if (config.isMounted()) {
            return config.getMount().getAngle() + Math.toRadians(config.getAngle());
        } else {
            return Math.toRadians(config.getAngle());
        }
    }

    /**
     * The X coordinate of this stage's static root, which a child attaches to.
     *
     * @return static root X position in metres
     */
    public double getMountX() {
        return config.getStaticRootX();
    }

    /**
     * The Y coordinate of this stage's static root, which a child attaches to.
     *
     * @return static root Y position in metres
     */
    public double getMountY() {
        return config.getStaticRootY();
    }
}
