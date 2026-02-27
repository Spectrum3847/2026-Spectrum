package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.rebuilt.ShotCalculator;
import frc.spectrumLib.sim.Circle;
import java.util.Set;
import lombok.Getter;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

// General Sim principles
// Always move the root/origin to change it's display position
// Looking at the robot from the left view (right side of the robot)
public class RobotSim {
    @Getter public static final double topViewHeight = 150;
    @Getter public static final double topViewWidth = 150;
    @Getter public static final double leftViewHeight = 75;
    @Getter public static final double leftViewWidth = 75;

    @Getter
    private static final IntakeSimulation intakeSimulation =
            IntakeSimulation.OverTheBumperIntake(
                    "Fuel",
                    Robot.getSwerve().getMapleSimSwerveDrivetrain().mapleSimDrive,
                    Inches.of(29),
                    Inches.of(12),
                    IntakeSimulation.IntakeSide.FRONT,
                    80);

    ;

    public static final Translation2d origin = new Translation2d(0.0, 0.0);

    public static final Mechanism2d topView =
            new Mechanism2d(
                    Units.inchesToMeters(topViewWidth), Units.inchesToMeters(topViewHeight));

    public static final Mechanism2d leftView =
            new Mechanism2d(
                    Units.inchesToMeters(leftViewWidth), Units.inchesToMeters(leftViewHeight));

    public RobotSim() {
        SmartDashboard.putData("Sim/TopView", RobotSim.topView);
        SmartDashboard.putData("Sim/LeftView", RobotSim.leftView);
        topView.setBackgroundColor(new Color8Bit(Color.kLightGray));
        leftView.setBackgroundColor(new Color8Bit(Color.kLightGray));

        drawRobot();
    }

    public void drawRobot() {
        drawTopRobot();
        drawSideRobot();
        drawTurretCircle();
    }

    @SuppressWarnings("unused")
    public void drawTurretCircle() {
        MechanismRoot2d circleRoot =
                topView.getRoot(
                        "Turret Circle Root",
                        Units.inchesToMeters(topViewHeight / 2 + 30),
                        Units.inchesToMeters(topViewWidth / 2));
        Circle circle = new Circle(50, 30, "Turret Circle", circleRoot, topView);
    }

    public void drawTopRobot() {
        MechanismRoot2d robotRoot =
                topView.getRoot(
                        "Top Robot Root",
                        Units.inchesToMeters(topViewWidth / 2.0 + 50),
                        Units.inchesToMeters(topViewHeight / 2.0 - 25));

        double rectWidthIn = 50.0;
        double rectHeightIn = 80.0;
        double rectWidthM = Units.inchesToMeters(rectWidthIn);
        double rectHeightM = Units.inchesToMeters(rectHeightIn);

        MechanismLigament2d tr =
                robotRoot.append(new MechanismLigament2d("TopEdge", rectHeightM, 180.0));
        MechanismLigament2d br = tr.append(new MechanismLigament2d("RightEdge", rectWidthM, 270.0));
        MechanismLigament2d bl = br.append(new MechanismLigament2d("BottomEdge", rectHeightM, 270));
        MechanismLigament2d ll = bl.append(new MechanismLigament2d("LeftEdge", rectWidthM, 270.0));

        Color8Bit edgeColor = new Color8Bit(Color.kPurple);
        tr.setColor(edgeColor);
        br.setColor(edgeColor);
        bl.setColor(edgeColor);
        ll.setColor(edgeColor);
    }

    public void drawSideRobot() {
        MechanismRoot2d robotRoot =
                leftView.getRoot(
                        "Top Robot Root",
                        Units.inchesToMeters(leftViewWidth / 2.0 + 25),
                        Units.inchesToMeters(leftViewHeight / 2.0 - 12.5));

        double rectWidthIn = 25.0;
        double rectHeightIn = 40.0;
        double rectWidthM = Units.inchesToMeters(rectWidthIn);
        double rectHeightM = Units.inchesToMeters(rectHeightIn);

        MechanismLigament2d tr =
                robotRoot.append(new MechanismLigament2d("TopEdge", rectHeightM, 180.0));
        MechanismLigament2d br = tr.append(new MechanismLigament2d("RightEdge", rectWidthM, 270.0));
        MechanismLigament2d bl = br.append(new MechanismLigament2d("BottomEdge", rectHeightM, 270));
        MechanismLigament2d ll = bl.append(new MechanismLigament2d("LeftEdge", rectWidthM, 270.0));

        Color8Bit edgeColor = new Color8Bit(Color.kPurple);
        tr.setColor(edgeColor);
        br.setColor(edgeColor);
        bl.setColor(edgeColor);
        ll.setColor(edgeColor);

        MechanismLigament2d shooter = bl.append(new MechanismLigament2d("shooter", 0.4, 135));
        shooter.setColor(new Color8Bit((Color.kBlack)));
    }

    // Maple Sim Fuel Intaking
    public static Command mapleSimIntakeFuel() {
        return new Command() {
            @Override
            public void initialize() {
                RobotSim.getIntakeSimulation().startIntake();
            }

            @Override
            public void execute() {
                SmartDashboard.putNumber(
                        "Sim/FuelCount", RobotSim.getIntakeSimulation().getGamePiecesAmount());
            }

            @Override
            public void end(boolean interrupted) {
                RobotSim.getIntakeSimulation().stopIntake();
            }
        };
    }

    // Maple Sim Fuel Projectile Creator
    public static Command mapleSimCreateFuelProjectile() {
        return new InstantCommand(
                () -> {
                    var parameters = ShotCalculator.getInstance().getParameters();
                    GamePieceProjectile fuelProjectile =
                            new RebuiltFuelOnFly(
                                            Robot.getSwerve().getRobotPose().getTranslation(),
                                            new Translation2d(),
                                            Robot.getSwerve().getCurrentRobotChassisSpeeds(),
                                            parameters.turretAngle(),
                                            Inches.of(29),
                                            MetersPerSecond.of(parameters.flywheelSpeed() * 0.0325),
                                            Degrees.of(65))
                                    .withProjectileTrajectoryDisplayCallBack(
                                            // Callback for when the fuel will eventually hit the
                                            // target (if configured)
                                            (pose3ds) ->
                                                    DogLog.log(
                                                            "SimShot/FuelProjectileSuccessfulShot",
                                                            pose3ds.toArray(Pose3d[]::new)),
                                            // Callback for when the fuel will eventually miss the
                                            // target, or if no target
                                            // is configured
                                            (pose3ds) ->
                                                    DogLog.log(
                                                            "SimShot/FuelProjectileUnsuccessfulShot",
                                                            pose3ds.toArray(Pose3d[]::new)));
                    SimulatedArena.getInstance().addGamePieceProjectile(fuelProjectile);
                    RobotSim.getIntakeSimulation().obtainGamePieceFromIntake();
                    SmartDashboard.putNumber(
                            "Sim/FuelCount", RobotSim.getIntakeSimulation().getGamePiecesAmount());
                });
    }

    public static Command mapleSimLaunchFuel() {
        return Commands.defer(
                () -> {
                    int fuelCount = RobotSim.getIntakeSimulation().getGamePiecesAmount();
                    SequentialCommandGroup group = new SequentialCommandGroup();
                    for (int i = 0; i < fuelCount; i++) {
                        group.addCommands(mapleSimCreateFuelProjectile(), new WaitCommand(0.125));
                    }
                    return group.withName("RobotSim.mapleSimLaunchFuel");
                },
                Set.of() // no subsystem requirements
                );
    }
}
