package frc.spectrumLib.sim;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * WPILib flywheel simulation of a roller on a single Kraken X60 motor. Call {@link
 * #simulationPeriodic()} once a period to step the physics, push the result into the TalonFX sim
 * state, and color the canvas by spin direction.
 *
 * <p>The roller is a {@link Mountable}, so its axle can follow a parent {@link Mount}.
 */
public class RollerSim implements Mountable {

    private MechanismRoot2d rollerAxle;
    private MechanismLigament2d rollerViz;

    private FlywheelSim rollerSim;
    private TalonFXSimState rollerMotorSim;
    private RollerConfig config;
    private Circle roller;

    public RollerSim(
            RollerConfig config, Mechanism2d mech, TalonFXSimState rollerMotorSim, String name) {
        this.config = config;
        this.rollerMotorSim = rollerMotorSim;
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
    }

    public void simulationPeriodic() {
        rollerSim.setInput(rollerMotorSim.getMotorVoltage());
        rollerSim.update(TimedRobot.kDefaultPeriod);

        // The sim reports mechanism-side radians, so the rotor count needs the gear ratio. The
        // starting angle comes off so the sim cannot be used as an absolute encoder.
        double rotorRotationsPerSecond =
                rollerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI) * config.getGearRatio();
        rollerMotorSim.setRotorVelocity(rotorRotationsPerSecond);
        rollerMotorSim.addRotorPosition(rotorRotationsPerSecond * TimedRobot.kDefaultPeriod);

        if (config.isMounted()) {
            rollerAxle.setPosition(getUpdatedX(config), getUpdatedY(config));
        } else {
            rollerAxle.setPosition(config.getInitialX(), config.getInitialY());
        }

        // Scaled down so the marker spins visibly instead of blurring.
        double rpm = rollerSim.getAngularVelocityRPM() / 2;
        rollerViz.setAngle(
                rollerViz.getAngle() + Math.toDegrees(rpm) * TimedRobot.kDefaultPeriod * 0.1);

        // Anything inside 1 rad/s counts as stopped.
        if (rollerSim.getAngularVelocityRadPerSec() < -1) {
            roller.setHalfBackground(config.getRevColor(), config.getOffColor());
        } else if (rollerSim.getAngularVelocityRadPerSec() > 1) {
            roller.setBackgroundColor(config.getFwdColor());
        } else {
            roller.setBackgroundColor(config.getOffColor());
        }
    }
}
