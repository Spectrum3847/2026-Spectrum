package frc.spectrumLib.sim;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;

/** Physical, display, and mount settings for a {@link RollerSim}. */
public class RollerConfig {
    @Getter private double rollerDiameterInches = 2;
    @Getter private int backgroundLines = 36;
    /** Motor rotations per roller rotation. */
    @Getter private double gearRatio = 5;
    /** Moment of inertia for the flywheel sim, in kg·m². */
    @Getter private double simMOI = 0.01;

    @Getter private Color8Bit offColor = new Color8Bit(Color.kBlack);
    @Getter private Color8Bit fwdColor = new Color8Bit(Color.kGreen);
    @Getter private Color8Bit revColor = new Color8Bit(Color.kRed);
    /** Initial axle X in metres. */
    @Getter private double initialX = 0;
    /** Initial axle Y in metres. */
    @Getter private double initialY = 0;

    @Getter private boolean mounted = false;
    @Getter private Mount mount;
    /** Mount X at the start of the run, in metres. */
    @Getter private double initMountX;
    /** Mount Y at the start of the run, in metres. */
    @Getter private double initMountY;
    /** Mount angle at the start of the run, in radians. */
    @Getter private double initMountAngle;

    public RollerConfig(double diameterInches) {
        rollerDiameterInches = diameterInches;
    }

    public RollerConfig setGearRatio(double ratio) {
        gearRatio = ratio;
        return this;
    }

    public RollerConfig setSimMOI(double moi) {
        simMOI = moi;
        return this;
    }

    public RollerConfig setPosition(double x, double y) {
        initialX = x;
        initialY = y;
        return this;
    }

    /**
     * Mounts the roller on a stage so its axle follows the carriage.
     *
     * @param sim the stage to mount onto, or {@code null} to leave this roller unmounted
     */
    public RollerConfig setMount(LinearSim sim) {
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
     * Mounts the roller on an arm so its axle follows the arm tip.
     *
     * @param sim the parent arm to mount onto, or {@code null} to leave this roller unmounted
     */
    public RollerConfig setMount(ArmSim sim) {
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
