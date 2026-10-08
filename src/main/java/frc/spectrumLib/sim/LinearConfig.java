package frc.spectrumLib.sim;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;
import lombok.Setter;

/**
 * Physical properties, Mechanism2d display settings, and the optional parent mount for a {@link
 * LinearSim}.
 */
public class LinearConfig implements Mountable.MountedConfig {
    /** Number of Kraken X60 motors driving the linear stage. */
    @Getter private int numMotors = 1;
    /** Gear ratio between the motor and the elevator drum. */
    @Getter private double elevatorGearing = 5;
    /** Mass of the moving carriage in kilograms, fed to the physics sim. */
    @Getter private double carriageMassKg = 1;
    /** Drum radius in metres, which converts motor rotations into carriage travel. */
    @Getter private double drumRadius = Units.inchesToMeters(0.955 / 2);
    /** Minimum travel height of the mechanism in metres. */
    @Getter private double minHeight = 0;
    /** Maximum travel height of the mechanism in metres. */
    @Getter private double maxHeight = 10000;

    /**
     * Angle of the linear stage on the Mechanism2d canvas, in degrees, where 0 is horizontal, 90 is
     * vertical, and positive is counter-clockwise.
     */
    @Getter private double angle = 90;

    @Getter private Color8Bit color = new Color8Bit(Color.kPurple);
    /** Stroke width of the stage ligaments, in pixels. */
    @Getter private double lineWidth = 10;
    /** Initial X of the static root on the canvas, in metres. */
    @Getter private double initialX = 0.5;
    /** Initial Y of the static root on the canvas, in metres. */
    @Getter private double initialY = 0;
    /** Current X of the static root in metres, moved every tick while mounted. */
    @Getter @Setter private double staticRootX = 0.5;
    /** Current Y of the static root in metres, moved every tick while mounted. */
    @Getter @Setter private double staticRootY = 0;
    /** Visual length of the non-moving stage ligament, in metres. */
    @Getter private double staticLength = 20;
    /** Visual length of the moving stage ligament, in metres. */
    @Getter private double movingLength = 20;
    /** The parent mount this linear stage is attached to, or {@code null} if not mounted. */
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

    public LinearConfig(double x, double y, double gearing, double drumRadius) {
        this.initialX = x;
        this.initialY = y;
        staticRootX = initialX;
        staticRootY = initialY;
        elevatorGearing = gearing;
        this.drumRadius = drumRadius;
    }

    /** True when the sim geometry travels opposite the motor's positive direction. */
    @Getter private boolean reversedLinkage = false;

    /**
     * Mirrors the simulated motor so the sim's travel matches the motor's positive direction. See
     * {@link SimMotor#simState(com.ctre.phoenix6.hardware.TalonFX, boolean)}.
     */
    public LinearConfig setReversedLinkage(boolean reversedLinkage) {
        this.reversedLinkage = reversedLinkage;
        return this;
    }

    public LinearConfig setNumMotors(int numMotors) {
        this.numMotors = numMotors;
        return this;
    }

    public LinearConfig setCarriageMass(double carriageMassKg) {
        this.carriageMassKg = carriageMassKg;
        return this;
    }

    public LinearConfig setAngle(double angle) {
        this.angle = angle;
        return this;
    }

    public LinearConfig setColor(Color8Bit color) {
        this.color = color;
        return this;
    }

    public LinearConfig setLineWidth(double lineWidth) {
        this.lineWidth = lineWidth;
        return this;
    }

    /** Sets the static ligament's visual length. Takes inches and stores metres. */
    public LinearConfig setStaticLength(double lengthInches) {
        this.staticLength = Units.inchesToMeters(lengthInches);
        return this;
    }

    /** Sets the moving ligament's visual length. Takes inches and stores metres. */
    public LinearConfig setMovingLength(double lengthInches) {
        this.movingLength = Units.inchesToMeters(lengthInches);
        return this;
    }

    /** Sets the top of the travel range. Takes inches and stores metres. */
    public LinearConfig setMaxHeight(double lengthInches) {
        this.maxHeight = Units.inchesToMeters(lengthInches);
        return this;
    }

    /**
     * Attaches this stage to a parent {@link LinearSim} so its root follows that stage.
     *
     * @param sim the stage to follow, or null to leave this one unmounted
     */
    public LinearConfig setMount(LinearSim sim) {
        if (sim != null) {
            mount = sim;
            initMountX = sim.getConfig().getInitialX();
            initMountY = sim.getConfig().getInitialY();
            initMountAngle = Math.toRadians(sim.getConfig().getAngle());
        }

        return this;
    }

    /**
     * Attaches this stage to a parent {@link ArmSim} so its root follows the arm tip.
     *
     * @param sim the arm to follow, or null to leave this stage unmounted
     */
    public LinearConfig setMount(ArmSim sim) {
        if (sim != null) {
            mount = sim;
            initMountX = sim.getConfig().getInitialX();
            initMountY = sim.getConfig().getInitialY();
            initMountAngle = sim.getConfig().getStartingAngle();
        }

        return this;
    }
}
