package frc.spectrumLib.sim;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;
import lombok.Setter;

/** Physical, display, and mount settings for a {@link LinearSim}. */
public class LinearConfig {
    @Getter private int numMotors = 1;
    /** Motor rotations per drum rotation. */
    @Getter private double elevatorGearing = 5;

    @Getter private double carriageMassKg = 1;
    /** Drum radius in metres, used to convert drum turns into travel. */
    @Getter private double drumRadius = Units.inchesToMeters(0.955 / 2);
    /** Minimum travel, in metres. */
    @Getter private double minHeight = 0;
    /** Maximum travel, in metres. Large enough to act as no limit. */
    @Getter private double maxHeight = 10000;

    /**
     * Stage angle in degrees, where 0 is horizontal and 90 is vertical, counter-clockwise positive.
     */
    @Getter private double angle = 90;

    @Getter private Color8Bit color = new Color8Bit(Color.kPurple);
    /** Stroke width in pixels. */
    @Getter private double lineWidth = 10;
    /** Initial static root X in metres. */
    @Getter private double initialX = 0.5;
    /** Initial static root Y in metres. */
    @Getter private double initialY = 0;
    /** Current static root X in metres, updated when mounted. */
    @Getter @Setter private double staticRootX = 0.5;
    /** Current static root Y in metres, updated when mounted. */
    @Getter @Setter private double staticRootY = 0;
    /** Visual length of the static ligament, in metres. */
    @Getter private double staticLength = 20;
    /** Visual length of the moving ligament, in metres. */
    @Getter private double movingLength = 20;

    @Getter private boolean mounted = false;
    @Getter private Mount mount;
    /** Mount X at the start of the run, in metres. */
    @Getter private double initMountX;
    /** Mount Y at the start of the run, in metres. */
    @Getter private double initMountY;
    /** Mount angle at the start of the run, in radians. */
    @Getter private double initMountAngle;

    /**
     * Builds a config for a stage with the given root, gearing, and drum.
     *
     * @param x stage root X in metres
     * @param y stage root Y in metres
     * @param gearing motor rotations per drum rotation
     * @param drumRadius drum radius in metres
     */
    public LinearConfig(double x, double y, double gearing, double drumRadius) {
        this.initialX = x;
        this.initialY = y;
        staticRootX = initialX;
        staticRootY = initialY;
        elevatorGearing = gearing;
        this.drumRadius = drumRadius;
    }

    public LinearConfig setNumMotors(int numMotors) {
        this.numMotors = numMotors;
        return this;
    }

    public LinearConfig setCarriageMass(double carriageMassKg) {
        this.carriageMassKg = carriageMassKg;
        return this;
    }

    /**
     * Sets how the stage is tilted on the canvas.
     *
     * @param angle stage angle in degrees, where 0 is horizontal and 90 is vertical
     */
    public LinearConfig setAngle(double angle) {
        this.angle = angle;
        return this;
    }

    public LinearConfig setColor(Color8Bit color) {
        this.color = color;
        return this;
    }

    /**
     * Sets how thick the stage ligaments draw.
     *
     * @param lineWidth stroke width in pixels
     */
    public LinearConfig setLineWidth(double lineWidth) {
        this.lineWidth = lineWidth;
        return this;
    }

    public LinearConfig setStaticLength(double lengthInches) {
        this.staticLength = Units.inchesToMeters(lengthInches);
        ;
        return this;
    }

    public LinearConfig setMovingLength(double lengthInches) {
        this.movingLength = Units.inchesToMeters(lengthInches);
        return this;
    }

    public LinearConfig setMaxHeight(double lengthInches) {
        this.maxHeight = Units.inchesToMeters(lengthInches);
        return this;
    }

    /**
     * Mounts this stage on a parent stage so its root follows along.
     *
     * @param sim the parent stage, or {@code null} to leave this stage unmounted
     */
    public LinearConfig setMount(LinearSim sim) {
        if (sim != null) {
            mounted = true;
            mount = sim;
            initMountX = sim.getConfig().getInitialX();
            initMountY = sim.getConfig().getInitialY();
            initMountAngle = Math.toRadians(sim.getConfig().getAngle());
        }

        return this;
    }

    /**
     * Mounts this stage on an arm so its root follows the arm tip.
     *
     * @param sim the parent arm, or {@code null} to leave this stage unmounted
     */
    public LinearConfig setMount(ArmSim sim) {
        if (sim != null) {
            mounted = true;
            mount = sim;
            initMountX = sim.getConfig().getInitialX();
            initMountY = sim.getConfig().getInitialY();
            initMountAngle = sim.getConfig().getStartingAngle();
        }

        return this;
    }
}
