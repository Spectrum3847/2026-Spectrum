package frc.spectrumLib.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import lombok.Getter;

/**
 * WPILib-backed simulation of a single-jointed arm driven by one or more Kraken X60 motors.
 * Advances the TalonFX sim state on the shared {@link SimLoop} thread and animates the arm in a
 * {@link Mechanism2d} canvas. Implements {@link Mount} so other mechanisms can attach to the arm
 * tip, and {@link Mountable} so this arm can itself attach to a parent mount.
 */
public class ArmSim implements Mount, Mountable {
    private SingleJointedArmSim armSim;
    @Getter private ArmConfig config;

    private MechanismRoot2d armPivot;
    private MechanismLigament2d armMech2d;
    private TalonFXSimState armMotorSim;

    /** Always {@link MountType#ARM}, so children apply arm placement. */
    @Getter private final MountType mountType = MountType.ARM;

    /**
     * @param mech the Mechanism2d canvas to draw the arm on
     * @param name prefix for this sim's Mechanism2d element labels
     */
    public ArmSim(ArmConfig config, Mechanism2d mech, TalonFX motor, String name) {
        this.config = config;
        this.armMotorSim = SimMotor.simState(motor, config.isReversedLinkage());
        armSim =
                new SingleJointedArmSim(
                        DCMotor.getKrakenX60Foc(config.getNumMotors()),
                        config.getRatio(),
                        config.getSimMOI(),
                        config.getSimCGLength(),
                        config.getMinAngle(),
                        config.getMaxAngle(),
                        config.isSimulateGravity(),
                        config.getStartingAngle());

        armPivot = mech.getRoot(name + " Arm Pivot", config.getPivotX(), config.getPivotY());
        armMech2d =
                armPivot.append(
                        new MechanismLigament2d(
                                name + " Arm",
                                config.getLength(),
                                config.getMinAngle(),
                                5.0,
                                config.getColor()));

        SimLoop.register(this::update);
    }

    /** One sim tick: steps the arm physics, feeds the rotor state back, and redraws. */
    public void update(double dt) {
        armSim.setInput(armMotorSim.getMotorVoltage());
        armSim.update(dt);

        armMotorSim.setRawRotorPosition(
                (Units.radiansToRotations(armSim.getAngleRads() - config.getStartingAngle()))
                        * config.getRatio());

        armMotorSim.setRotorVelocity(
                Units.radiansToRotations(armSim.getVelocityRadPerSec()) * config.getRatio());

        if (config.isMounted()) {
            config.setPivotX(getUpdatedX(config));
            config.setPivotY(getUpdatedY(config));
        }
        armMech2d.setAngle(Math.toDegrees(getAngle()));

        armPivot.setPosition(config.getPivotX(), config.getPivotY());
    }

    public double getAngleRads() {
        return armSim.getAngleRads();
    }

    /**
     * How far the pivot has moved horizontally from its initial position.
     *
     * @return horizontal displacement in metres
     */
    public double getDisplacementX() {
        return config.getPivotX() - config.getInitialX();
    }

    /**
     * How far the pivot has moved vertically from its initial position.
     *
     * @return vertical displacement in metres
     */
    public double getDisplacementY() {
        return config.getPivotY() - config.getInitialY();
    }

    /**
     * The arm's own angle, plus the parent mount's angle unless the config asked for an absolute
     * angle.
     *
     * @return effective arm angle in radians
     */
    public double getAngle() {
        return config.isMounted() && !config.isAbsAngle()
                ? getAngleRads() + config.getMount().getAngle()
                : getAngleRads();
    }

    /**
     * The X coordinate of the arm pivot, which a child attaches to.
     *
     * @return pivot X position in metres
     */
    public double getMountX() {
        return config.getPivotX();
    }

    /**
     * The Y coordinate of the arm pivot, which a child attaches to.
     *
     * @return pivot Y position in metres
     */
    public double getMountY() {
        return config.getPivotY();
    }
}
