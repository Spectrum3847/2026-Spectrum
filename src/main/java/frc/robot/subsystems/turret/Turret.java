package frc.robot.subsystems.turret;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.telemetry.*;
import frc.spectrumLib.mechanism.Mechanism;
import lombok.Getter;
import lombok.Setter;

public class Turret extends Mechanism {

    public static class TurretConfig extends Config {
        
    }

}