package frc.spectrumLib.sim;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;

/**
 * Physical properties, display colors, canvas position, and the optional parent mount for a {@link
 * RollerSim}.
 */
public class RollerConfig implements Mountable.MountedConfig {
    /** Roller diameter in inches, used for both the physics model and the drawing. */
    @Getter private double rollerDiameterInches = 2;

    @Getter private int backgroundLines = 36;
    /** Motor rotations per roller revolution. */
    @Getter private double gearRatio = 5;
    /** Roller moment of inertia in kg·m², fed to the flywheel sim. */
    @Getter private double simMOI = 0.01;
    /** Color below the spin threshold the sim treats as stationary. */
    @Getter private Color8Bit offColor = new Color8Bit(Color.kBlack);
    /** Color while the roller spins in the forward direction. */
    @Getter private Color8Bit fwdColor = new Color8Bit(Color.kGreen);
    /** Color while the roller spins in the reverse direction. */
    @Getter private Color8Bit revColor = new Color8Bit(Color.kRed);
    /** Initial axle X on the Mechanism2d canvas, in metres. */
    @Getter private double initialX = 0;
    /** Initial axle Y on the Mechanism2d canvas, in metres. */
    @Getter private double initialY = 0;
    /** The parent mount this roller is attached to, or {@code null} if not mounted. */
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

    public RollerConfig(double diameterInches) {
        rollerDiameterInches = diameterInches;
    }

    /** True when the sim geometry travels opposite the motor's positive direction. */
    @Getter private boolean reversedLinkage = false;

    /**
     * Mirrors the simulated motor so the sim's travel matches the motor's positive direction. See
     * {@link SimMotor#simState(com.ctre.phoenix6.hardware.TalonFX, boolean)}.
     */
    public RollerConfig setReversedLinkage(boolean reversedLinkage) {
        this.reversedLinkage = reversedLinkage;
        return this;
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
     * Attaches this roller to a {@link LinearSim} so its axle follows that stage.
     *
     * @param sim the stage to follow, or null to leave this roller unmounted
     */
    public RollerConfig setMount(LinearSim sim) {
        if (sim != null) {
            mount = sim;
            initMountX = sim.getConfig().getInitialX();
            initMountY = sim.getConfig().getInitialY();
            initMountAngle = Math.toRadians(sim.getConfig().getAngle());
        }

        return this;
    }

    /**
     * Attaches this roller to an {@link ArmSim} so its axle follows the arm tip.
     *
     * @param sim the arm to follow, or null to leave this roller unmounted
     */
    public RollerConfig setMount(ArmSim sim) {
        if (sim != null) {
            mount = sim;
            initMountX = sim.getConfig().getInitialX();
            initMountY = sim.getConfig().getInitialY();
            initMountAngle = sim.getConfig().getStartingAngle();
        }

        return this;
    }
}
