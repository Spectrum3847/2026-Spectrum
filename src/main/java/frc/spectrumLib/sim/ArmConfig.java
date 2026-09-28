package frc.spectrumLib.sim;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;
import lombok.Setter;

/** Physical, display, and mount settings for an {@link ArmSim}. */
public class ArmConfig {

    @Getter @Setter private int numMotors = 1;
    /** Initial pivot X in metres. */
    @Getter @Setter private double initialX;
    /** Initial pivot Y in metres. */
    @Getter @Setter private double initialY;
    /** Current pivot X in metres. */
    @Getter @Setter private double pivotX;
    /** Current pivot Y in metres. */
    @Getter @Setter private double pivotY;
    /** Motor rotations per arm revolution. */
    @Getter @Setter private double ratio;
    /** Visual length of the arm ligament, in metres. */
    @Getter @Setter private double length;
    /** Moment of inertia for the physics sim, in kg·m². */
    @Getter @Setter private double simMOI = 1.2;
    /** Pivot to center of gravity, in metres. */
    @Getter @Setter private double simCGLength = 0.2;
    /** Minimum arm angle, in radians. */
    @Getter @Setter private double minAngle;
    /** Maximum arm angle, in radians. */
    @Getter @Setter private double maxAngle;
    /** Arm angle at the start of the run, in radians. */
    @Getter @Setter private double startingAngle;

    @Getter @Setter private boolean simulateGravity = true;
    @Getter private boolean mounted = false;
    @Getter private Mount mount;
    /** Mount X at the start of the run, in metres. */
    @Getter private double initMountX;
    /** Mount Y at the start of the run, in metres. */
    @Getter private double initMountY;
    /** Mount angle at the start of the run, in radians. */
    @Getter private double initMountAngle;
    /**
     * When true the visual angle is absolute; when false it is measured from the parent mount's
     * current angle.
     */
    @Getter private boolean absAngle;

    @Getter private Color8Bit color = new Color8Bit(Color.kBlue);

    /** Angle arguments are in degrees and are stored in radians. */
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

    public ArmConfig setColor(Color8Bit color) {
        this.color = color;
        return this;
    }

    public ArmConfig setSimulatedGravity(boolean simulateGravity) {
        this.simulateGravity = simulateGravity;
        return this;
    }

    /**
     * Mounts the arm on a linear stage so its pivot follows the carriage.
     *
     * @param sim the stage to mount onto, or {@code null} to leave this arm unmounted
     * @param fixedAngle when true the visual angle is absolute, otherwise it is measured from the
     *     stage's current angle
     */
    public ArmConfig setMount(LinearSim sim, boolean fixedAngle) {
        if (sim != null) {
            mounted = true;
            mount = sim;
            initMountX = sim.getConfig().getInitialX();
            initMountY = sim.getConfig().getInitialY();
            initMountAngle = Math.toRadians(sim.getConfig().getAngle());
            this.absAngle = fixedAngle;
        }
        return this;
    }

    /**
     * Mounts the arm on a parent arm so its pivot follows that arm's tip.
     *
     * @param sim the parent arm to mount onto, or {@code null} to leave this arm unmounted
     * @param absAngle when true the visual angle is absolute, otherwise it is measured from the
     *     parent arm's current angle
     */
    public ArmConfig setMount(ArmSim sim, boolean absAngle) {
        if (sim != null) {
            mounted = true;
            mount = sim;
            initMountX = sim.getConfig().getInitialX();
            initMountY = sim.getConfig().getInitialY();
            initMountAngle = sim.getConfig().getStartingAngle();
            this.absAngle = absAngle;
        }
        return this;
    }
}
