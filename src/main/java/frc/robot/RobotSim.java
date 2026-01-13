package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.spectrumLib.sim.Circle;

// General Sim principles
// Always move the root/origin to change it's display position
// Looking at the robot from the left view (right side of the robot)
public class RobotSim {
    public static final double height = 150;
    public static final double width = 150;

    public static final Translation2d origin =
            new Translation2d(Units.inchesToMeters(width / 2), 0.0);

    public static final Mechanism2d topView =
            new Mechanism2d(Units.inchesToMeters(width), Units.inchesToMeters(height));

    public RobotSim() {
        SmartDashboard.putData("TopView", RobotSim.topView);
        topView.setBackgroundColor(new Color8Bit(Color.kLightGray));

        drawTurretCircle();
    }
    @SuppressWarnings("unused")
    public void drawTurretCircle() {
        MechanismRoot2d circleRoot = topView.getRoot("Turret Circle Root", 2, 2);
        Circle circle = new Circle(50, 40, "Turret Circle", circleRoot, topView);
    }
}
