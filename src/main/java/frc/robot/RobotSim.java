package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.rebuilt.ShotCalculator;
import frc.spectrumLib.sim.Circle;
import lombok.Getter;

// General Sim principles
// Always move the root/origin to change it's display position
// Looking at the robot from the left view (right side of the robot)
public class RobotSim {
    @Getter public static final double topViewHeight = 150;
    @Getter public static final double topViewWidth = 150;
    @Getter public static final double leftViewHeight = 75;
    @Getter public static final double leftViewWidth = 75;

    public static final Translation2d origin =
            new Translation2d(0.0, 0.0);

    public static final Mechanism2d topView =
            new Mechanism2d(Units.inchesToMeters(topViewWidth), Units.inchesToMeters(topViewHeight));

    public static final Mechanism2d leftView =
            new Mechanism2d(Units.inchesToMeters(leftViewWidth), Units.inchesToMeters(leftViewHeight));

    public RobotSim() {
        SmartDashboard.putData("TopView", RobotSim.topView);
        SmartDashboard.putData("LeftView", RobotSim.leftView);
        topView.setBackgroundColor(new Color8Bit(Color.kLightGray));
        leftView.setBackgroundColor(new Color8Bit(Color.kLightGray));

        drawTurretCircle();
    }

    @SuppressWarnings("unused")
    public void drawTurretCircle() {
        MechanismRoot2d circleRoot = topView.getRoot("Turret Circle Root", Units.inchesToMeters(topViewHeight / 2), Units.inchesToMeters(topViewWidth / 2));
        Circle circle = new Circle(50, 40, "Turret Circle", circleRoot, topView);
    }

    // Maple Sim Fuel Scoring
    public static Command mapleSimLaunchFuel() {
        return new InstantCommand(
                () -> {
                    var parameters = ShotCalculator.getInstance().getParameters();
                    SimulatedArena.getInstance()
                            .addGamePieceProjectile(
                                    new RebuiltFuelOnFly(
                                            Robot.getSwerve().getRobotPose().getTranslation(),
                                            new Translation2d(),
                                            Robot.getSwerve().getCurrentRobotChassisSpeeds(),
                                            parameters.turretAngle(),
                                            Inches.of(25),
                                            MetersPerSecond.of(parameters.flywheelSpeed() * 0.0325),
                                            Radians.of(parameters.hoodAngle()))
                                                .withProjectileTrajectoryDisplayCallBack(
                                                        // Callback for when the fuel will eventually hit the target (if configured)
                                                        (pose3ds) -> DogLog.log("SimShot/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
                                                        // Callback for when the fuel will eventually miss the target, or if no target is configured
                                                        (pose3ds) -> DogLog.log("SimShot/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new))
                                                        ));
                });
    }
}
