package frc.spectrumLib.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * WPILib-backed simulation of a roller (flywheel) mechanism driven by a single Kraken X60 motor.
 * Advances the TalonFX sim state on the shared {@link SimLoop} thread and animates the roller in a
 * {@link Mechanism2d} canvas, coloring by spin direction. Implements {@link Mountable} so the
 * roller axle can follow a parent {@link Mount}.
 */
public class RollerSim implements Mountable {

    private MechanismRoot2d rollerAxle;
    private MechanismLigament2d rollerViz;

    private FlywheelSim rollerSim;
    private TalonFXSimState rollerMotorSim;
    private RollerConfig config;
    private Circle roller;

    /**
     * @param mech the Mechanism2d canvas to draw the roller on
     * @param name prefix for this sim's Mechanism2d element labels
     */
    public RollerSim(RollerConfig config, Mechanism2d mech, TalonFX motor, String name) {
        this.config = config;
        this.rollerMotorSim = SimMotor.simState(motor, config.isReversedLinkage());
        DCMotor kraken = DCMotor.getKrakenX60Foc(1);
        LinearSystem<N1, N1, N1> flyWheelSystem =
                LinearSystemId.createFlywheelSystem(
                        kraken, config.getSimMOI(), config.getGearRatio());
        rollerSim = new FlywheelSim(flyWheelSystem, kraken);

        rollerAxle = mech.getRoot(name + " Axle", 0.0, 0.0);

        rollerViz =
                rollerAxle.append(
                        new MechanismLigament2d(
                                name + " Roller",
                                Units.inchesToMeters(config.getRollerDiameterInches()) / 2.0,
                                0.0,
                                5.0,
                                new Color8Bit(Color.kWhite)));

        roller =
                new Circle(
                        config.getBackgroundLines(),
                        config.getRollerDiameterInches(),
                        name,
                        rollerAxle,
                        mech);

        SimLoop.register(this::update);
    }

    /** One sim tick: steps the flywheel, feeds the rotor state back, and redraws. */
    public void update(double dt) {
        rollerSim.setInput(rollerMotorSim.getMotorVoltage());
        rollerSim.update(dt);

        // FlywheelSim reports mechanism-side velocity; the rotor turns gearRatio times faster.
        double rotorRotationsPerSecond =
                rollerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI) * config.getGearRatio();
        rollerMotorSim.setRotorVelocity(rotorRotationsPerSecond);
        rollerMotorSim.addRotorPosition(rotorRotationsPerSecond * dt);

        if (config.isMounted()) {
            rollerAxle.setPosition(getUpdatedX(config), getUpdatedY(config));
        } else {
            rollerAxle.setPosition(config.getInitialX(), config.getInitialY());
        }

        // Scale the drawn angle down so a spinning roller reads as a blur rather than a strobe.
        double rpm = rollerSim.getAngularVelocityRPM() / 2;
        rollerViz.setAngle(rollerViz.getAngle() + Math.toDegrees(rpm) * dt * 0.1);

        if (rollerSim.getAngularVelocityRadPerSec() < -1) {
            roller.setHalfBackground(config.getRevColor(), config.getOffColor());
        } else if (rollerSim.getAngularVelocityRadPerSec() > 1) {
            roller.setBackgroundColor(config.getFwdColor());
        } else {
            roller.setBackgroundColor(config.getOffColor());
        }
    }
}
