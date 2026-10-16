package frc.spectrumLib.sim;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;
import lombok.Setter;

/** Physical properties, display settings, and the optional parent mount for an {@link ArmSim}. */
public class ArmConfig implements Mountable.MountedConfig {

    /** Number of Kraken X60 motors driving the arm. */
    @Getter @Setter private int numMotors = 1;
    /** Initial pivot X on the Mechanism2d canvas, in metres. */
    @Getter @Setter private double initialX;
    /** Initial pivot Y on the Mechanism2d canvas, in metres. */
    @Getter @Setter private double initialY;
    /** Current pivot X in metres, moved every tick while mounted. */
    @Getter @Setter private double pivotX;
    /** Current pivot Y in metres, moved every tick while mounted. */
    @Getter @Setter private double pivotY;
    /** Motor rotations required for one full revolution of the arm mechanism. */
    @Getter @Setter private double ratio;
    /** Visual length of the arm ligament, in metres. */
    @Getter @Setter private double length;
    /** Moment of inertia used by the physics simulation (kg·m²). */
    @Getter @Setter private double simMOI = 1.2;
    /**
     * Distance from the pivot to the arm's centre of gravity used by the physics simulation
     * (metres).
     */
    @Getter @Setter private double simCGLength = 0.2;
    /** Minimum allowable arm angle (radians). */
    @Getter @Setter private double minAngle;
    /** Maximum allowable arm angle (radians). */
    @Getter @Setter private double maxAngle;
    /** Arm angle at the start of the simulation (radians). */
    @Getter @Setter private double startingAngle;

    @Getter @Setter private boolean simulateGravity = true;
    /** The parent mount this arm is attached to, or {@code null} if not mounted. */
    @Getter private Mount mount;

    public boolean isMounted() {
        return mount != null;
    }

    /** X position of the mount at simulation start (metres). */
    @Getter private double initMountX;
    /** Y position of the mount at simulation start (metres). */
    @Getter private double initMountY;
    /** Angle of the mount at simulation start (radians). */
    @Getter private double initMountAngle;
    /**
     * True keeps the arm's own angle absolute in the robot frame. False adds the parent mount's
     * angle to it, in radians.
     */
    @Getter private boolean absAngle;
    /** Color used to draw the arm ligament in the Mechanism2d canvas. */
    @Getter private Color8Bit color = new Color8Bit(Color.kBlue);

    /**
     * Angle arguments arrive in degrees and are stored in radians.
     *
     * @param ratio motor rotations per one full arm revolution
     */
    public ArmConfig(
            double initialX,
            double initialY,
            double ratio,
            double length,
            double minAngleDegrees,
            double maxAngleDegrees,
            double startingAngleDegrees) {
        this.ratio = ratio;
        this.length = length;
        this.minAngle = Math.toRadians(minAngleDegrees);
        this.maxAngle = Math.toRadians(maxAngleDegrees);
        this.startingAngle = Math.toRadians(startingAngleDegrees);
        this.initialX = initialX;
        this.initialY = initialY;
        this.pivotX = initialX;
        this.pivotY = initialY;
    }

    /** True when the sim geometry travels opposite the motor's positive direction. */
    @Getter private boolean reversedLinkage = false;

    /**
     * Mirrors the simulated motor so the sim's travel matches the motor's positive direction. See
     * {@link SimMotor#simState(com.ctre.phoenix6.hardware.TalonFX, boolean)}.
     */
    public ArmConfig setReversedLinkage(boolean reversedLinkage) {
        this.reversedLinkage = reversedLinkage;
        return this;
    }

    public ArmConfig setColor(Color8Bit color) {
        this.color = color;
        return this;
    }

    public ArmConfig setSimulatedGravity(boolean simulateGravity) {
        this.simulateGravity = simulateGravity;
        return this;
    }

    /**
     * Attaches this arm to a {@link LinearSim} so its pivot follows that stage.
     *
     * @param sim the stage to follow, or null to leave this arm unmounted
     * @param fixedAngle true treats the arm angle as absolute, false adds the mount's current angle
     */
    public ArmConfig setMount(LinearSim sim, boolean fixedAngle) {
        if (sim != null) {
            mount = sim;
            initMountX = sim.getConfig().getInitialX();
            initMountY = sim.getConfig().getInitialY();
            initMountAngle = Math.toRadians(sim.getConfig().getAngle());
            this.absAngle = fixedAngle;
        }
        return this;
    }

    /**
     * Attaches this arm to a parent {@link ArmSim} so its pivot follows that arm's tip.
     *
     * @param sim the arm to follow, or null to leave this arm unmounted
     * @param absAngle true treats the arm angle as absolute, false adds the parent's current angle
     */
    public ArmConfig setMount(ArmSim sim, boolean absAngle) {
        if (sim != null) {
            mount = sim;
            initMountX = sim.getConfig().getInitialX();
            initMountY = sim.getConfig().getInitialY();
            initMountAngle = sim.getConfig().getStartingAngle();
            this.absAngle = absAngle;
        }
        return this;
    }
}
