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
 * WPILib-backed simulation of a roller (flywheel) mechanism driven by a single Kraken X60 motor.
 * Updates the TalonFX sim state each robot period and animates the roller — including spin-color
 * feedback — in a {@link Mechanism2d} canvas. Implements {@link Mountable} so the roller axle can
 * follow a parent {@link Mount}.
 */
public class RollerSim implements Mountable {

    private MechanismRoot2d rollerAxle;
    private MechanismLigament2d rollerViz;

    private FlywheelSim rollerSim;
    private TalonFXSimState rollerMotorSim;
    private RollerConfig config;
    private Circle roller;

    /**
     * Creates and registers a roller simulation.
     *
     * @param config physical and display configuration for the roller
     * @param mech the Mechanism2d canvas to draw the roller on
     * @param rollerMotorSim the TalonFX sim state of the motor driving the roller
     * @param name unique name prefix used for Mechanism2d element labels
     */
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

    /**
     * Advances the flywheel physics simulation by one robot period, updates the TalonFX rotor
     * velocity and position, moves the axle to its current mount position, and updates the
     * Mechanism2d color to reflect the roller's spin direction.
     */
    public void simulationPeriodic() { // double x, double y) {
        // ------ Update sim based on motor output
        rollerSim.setInput(rollerMotorSim.getMotorVoltage());
        rollerSim.update(TimedRobot.kDefaultPeriod);

        // ------ Update motor based on sim
        // Make sure to convert radians at the mechanism to rotations at the motor
        // Subtracting out the starting angle is necessary so the simulation can't "cheat" and use
        // the
        // sim as an absolute encoder.
        double rotationsPerSecond = rollerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
        rollerMotorSim.setRotorVelocity(rotationsPerSecond);
        rollerMotorSim.addRotorPosition(rotationsPerSecond * TimedRobot.kDefaultPeriod);

        // Update the axle as the robot moves
        if (config.isMounted()) {
            rollerAxle.setPosition(getUpdatedX(config), getUpdatedY(config));
        } else {
            rollerAxle.setPosition(config.getInitialX(), config.getInitialY());
        }

        // Scale down the angular velocity so we can actually see what is happening
        double rpm = rollerSim.getAngularVelocityRPM() / 2;
        rollerViz.setAngle(
                rollerViz.getAngle() + Math.toDegrees(rpm) * TimedRobot.kDefaultPeriod * 0.1);

        if (rollerSim.getAngularVelocityRadPerSec() < -1) {
            roller.setHalfBackground(config.getRevColor(), config.getOffColor());
        } else if (rollerSim.getAngularVelocityRadPerSec() > 1) {
            roller.setBackgroundColor(config.getFwdColor());
        } else {
            roller.setBackgroundColor(config.getOffColor());
        }
    }
}
