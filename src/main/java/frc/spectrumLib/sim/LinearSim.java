package frc.spectrumLib.sim;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;

public class LinearSim implements Mount, Mountable {
    private ElevatorSim elevatorSim;

    private final MechanismRoot2d staticRoot;
    private final MechanismRoot2d root;
    private final MechanismLigament2d staticMech2d;
    private final MechanismLigament2d m_elevatorMech2d;
    @Getter private LinearConfig config;
    private TalonFXSimState linearMotorSim;

    @Getter private final MountType mountType = MountType.LINEAR;

    public LinearSim(
            LinearConfig config, Mechanism2d mech, TalonFXSimState linearMotorSim, String name) {
        this.config = config;
        this.linearMotorSim = linearMotorSim;

        this.elevatorSim =
                new ElevatorSim(
                        DCMotor.getKrakenX60Foc(config.getNumMotors()),
                        config.getElevatorGearing(),
                        config.getCarriageMassKg(),
                        config.getDrumRadius(),
                        config.getMinHeight(),
                        config.getMaxHeight(),
                        true,
                        0);

        staticRoot =
                mech.getRoot(name + " 1StaticRoot", config.getInitialX(), config.getInitialY());
        staticMech2d =
                staticRoot.append(
                        new MechanismLigament2d(
                                name + " 1Static",
                                config.getStaticLength(),
                                config.getAngle(),
                                config.getLineWidth(),
                                new Color8Bit(Color.kOrange)));

        root = mech.getRoot(name + " Root", config.getInitialX(), config.getInitialY());
        m_elevatorMech2d =
                root.append(
                        new MechanismLigament2d(
                                name,
                                config.getMovingLength(),
                                config.getAngle(),
                                config.getLineWidth(),
                                new Color8Bit(Color.kBlack)));
    }

    public MechanismLigament2d getElevatorMech2d() {
        return m_elevatorMech2d;
    }

    private double getRotationPerSec() {
        return (elevatorSim.getVelocityMetersPerSecond() / (2 * Math.PI * config.getDrumRadius()))
                * config.getElevatorGearing();
    }

    private double getRotations() {
        return (elevatorSim.getPositionMeters() / (2 * Math.PI * config.getDrumRadius()))
                * config.getElevatorGearing();
    }

    public void simulationPeriodic() {
        elevatorSim.setInput(linearMotorSim.getMotorVoltage());
        elevatorSim.update(TimedRobot.kDefaultPeriod);

        linearMotorSim.setRotorVelocity(getRotationPerSec());
        linearMotorSim.setRawRotorPosition(getRotations());

        double displacement = elevatorSim.getPositionMeters();

        if (config.isMounted()) {
            double angle;

            if (config.getMount().getMountType() == MountType.ARM) {
                angle = config.getAngle() + Math.toDegrees(config.getMount().getAngle());
            } else if (config.getMount().getMountType() == MountType.LINEAR) {
                angle =
                        config.getAngle()
                                + Math.toDegrees(
                                        config.getMount().getAngle() - config.getInitMountAngle());
            } else {
                angle = config.getAngle();
            }

            config.setStaticRootX(getUpdatedX(config));
            config.setStaticRootY(getUpdatedY(config));

            staticRoot.setPosition(config.getStaticRootX(), config.getStaticRootY());
            root.setPosition(
                    config.getStaticRootX() + (displacement * Math.cos(Math.toRadians(angle))),
                    config.getStaticRootY() + (displacement * Math.sin(Math.toRadians(angle))));

            staticMech2d.setAngle(angle);
            m_elevatorMech2d.setAngle(angle);

        } else {
            root.setPosition(
                    config.getInitialX()
                            + (displacement * Math.cos(Math.toRadians(config.getAngle()))),
                    config.getInitialY()
                            + (displacement * Math.sin(Math.toRadians(config.getAngle()))));
        }
    }

    public double getDisplacementX() {
        double angle;

        if (!config.isMounted()) {
            angle = config.getAngle();
        } else if (config.getMount().getMountType() == MountType.ARM) {
            angle = config.getAngle() + Math.toDegrees(config.getMount().getAngle());
        } else if (config.getMount().getMountType() == MountType.LINEAR) {
            angle =
                    config.getAngle()
                            + Math.toDegrees(
                                    config.getMount().getAngle() - config.getInitMountAngle());
        } else {
            angle = config.getAngle();
        }

        return elevatorSim.getPositionMeters() * Math.cos(Math.toRadians(angle))
                + (config.getStaticRootX() - config.getInitialX());
    }

    public double getDisplacementY() {
        double angle;

        if (!config.isMounted()) {
            angle = config.getAngle();
        } else if (config.getMount().getMountType() == MountType.ARM) {
            angle = config.getAngle() + Math.toDegrees(config.getMount().getAngle());
        } else if (config.getMount().getMountType() == MountType.LINEAR) {
            angle =
                    config.getAngle()
                            + Math.toDegrees(
                                    config.getMount().getAngle() - config.getInitMountAngle());
        } else {
            angle = config.getAngle();
        }

        return elevatorSim.getPositionMeters() * Math.sin(Math.toRadians(angle))
                + (config.getStaticRootY() - config.getInitialY());
    }

    public double getAngle() {
        if (config.isMounted()) {
            return config.getMount().getAngle() + Math.toRadians(config.getAngle());
        } else {
            return Math.toRadians(config.getAngle());
        }
    }

    public double getMountX() {
        return config.getStaticRootX();
    }

    public double getMountY() {
        return config.getStaticRootY();
    }
}
