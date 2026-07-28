package frc.robot;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.rebuilt.FuelPhysicsSim;
import frc.rebuilt.ShotCalculator;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.CurrentSuperState;
import frc.spectrumLib.sim.Circle;
import frc.spectrumLib.telemetry.Telemetry;
import java.util.Set;
import lombok.Getter;

// General Sim principles
// Always move the root/origin to change it's display position
// Looking at the robot from the left view (right side of the robot)
public class RobotSim {
    @Getter public static final double topViewHeight = 150;
    @Getter public static final double topViewWidth = 150;
    @Getter public static final double leftViewHeight = 75;
    @Getter public static final double leftViewWidth = 75;

    public static final Translation2d origin = new Translation2d(0.0, 0.0);

    public static final Mechanism2d leftView =
            new Mechanism2d(
                    Units.inchesToMeters(leftViewWidth), Units.inchesToMeters(leftViewHeight));

    public static final Mechanism2d topView =
            new Mechanism2d(
                    Units.inchesToMeters(topViewWidth), Units.inchesToMeters(topViewHeight));

    @Getter private static double simRobotWidth = Units.inchesToMeters(31.36);
    @Getter private static double simRobotLength = Units.inchesToMeters(37.86);

    private static final Translation3d TURRET_PIVOT_POINT = Translation3d.kZero;
    private static final Translation3d HOOD_PIVOT_POINT = new Translation3d(0.0, 0.0, 0.487);
    private static final double INTAKE_DEPLOY_ANGLE_DEGREES = -12;

    @Getter private FuelPhysicsSim ballSim;
    private double ballsPerSecond = 15;
    private double timeBetweenBallLaunches = 1.0 / ballsPerSecond;
    private static final double LAUNCH_HEIGHT_INCHES = 19.5;

    private SuperStructure robotSuperStructure;

    public RobotSim(SuperStructure superStructure) {
        this.robotSuperStructure = superStructure;
        SmartDashboard.putData("Sim/LeftView", RobotSim.leftView);
        SmartDashboard.putData("Sim/TopView", RobotSim.topView);
        leftView.setBackgroundColor(new Color8Bit(Color.kLightGray));
        topView.setBackgroundColor(new Color8Bit(Color.kLightGray));
        drawRobot();

        ballSim = new FuelPhysicsSim("Sim/Fuel");
        ballSim.enable();
        ballSim.placeFieldBalls(); // spawns all the game pieces
        configBallSimRobot();
    }

    public void updateArticulatedMechanisms() {
        double intakeExtensionMeters =
                Units.inchesToMeters(12)
                        * robotSuperStructure.getIntakeExtension().getPositionPercentage()
                        / 100;
        var intakeDeployRotation =
                new Rotation3d(0, -Math.toRadians(INTAKE_DEPLOY_ANGLE_DEGREES), 0);
        var intakePose3d =
                Pose3d.kZero.plus(
                        new Transform3d(
                                new Translation3d(intakeExtensionMeters, 0, 0)
                                        .rotateBy(intakeDeployRotation),
                                Rotation3d.kZero));

        double turretAngleDegrees =
                robotSuperStructure.getTurret().getPositionDegrees()
                        + robotSuperStructure
                                .getTurret()
                                .getConfig()
                                .getZeroOffsetFromRobotFront()
                                .getDegrees();
        var turretPose3d =
                Pose3d.kZero.rotateAround(
                        TURRET_PIVOT_POINT,
                        new Rotation3d(0, 0, Math.toRadians(turretAngleDegrees)));

        double hoodAngleDegrees = robotSuperStructure.getHood().getPositionDegrees();
        var hoodPose3d =
                Pose3d.kZero
                        .rotateAround(
                                HOOD_PIVOT_POINT,
                                new Rotation3d(0, Math.toRadians(hoodAngleDegrees), 0))
                        .rotateBy(new Rotation3d(0, 0, Math.toRadians(turretAngleDegrees)));

        double dyeRotorDegrees = robotSuperStructure.getDyeRotor().getRotor().getPositionDegrees();
        var dyeRotorPose3d =
                Pose3d.kZero.rotateAround(
                        TURRET_PIVOT_POINT, new Rotation3d(0, 0, Math.toRadians(dyeRotorDegrees)));

        Pose3d[] mechanismPoses = {intakePose3d, turretPose3d, hoodPose3d, dyeRotorPose3d};

        Telemetry.log("Sim/Components", mechanismPoses);
    }

    public void drawRobot() {
        drawSideRobot();
        drawTopRobot();
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

        // MechanismLigament2d shooter = bl.append(new MechanismLigament2d("shooter", 0.4, 135));
        // shooter.setColor(new Color8Bit(Color.kBlack));
    }

    private void configBallSimRobot() {
        double bumperHeight = Units.inchesToMeters(5);
        double intakeWidth = Units.inchesToMeters(8.275);
        double intakeLength = Units.inchesToMeters(32.2);
        double intakeXMin = Units.inchesToMeters(0);
        double intakeXMax = simRobotWidth / 2 + intakeWidth;
        double intakeYMin = -intakeLength / 2;
        double intakeYMax = intakeLength / 2;
        ballSim.configureRobot(
                simRobotWidth,
                simRobotLength,
                bumperHeight,
                80,
                () -> robotSuperStructure.getSwerve().getRobotPose(),
                () -> robotSuperStructure.getSwerve().getCurrentRobotChassisSpeeds());
        ballSim.addIntakeZone(
                intakeXMin,
                intakeXMax,
                intakeYMin,
                intakeYMax,
                () -> robotSuperStructure.getCurrentSuperState() == CurrentSuperState.INTAKE_FUEL);
    }

    private Command createSimBallLaunch() {
        return Commands.runOnce(
                () -> {
                    var params = ShotCalculator.getInstance().getParameters();
                    var turret = robotSuperStructure.getTurret();
                    Pose2d robotPos = robotSuperStructure.getSwerve().getRobotPose();

                    Rotation2d turretFieldAngle =
                            robotPos.getRotation()
                                    .plus(turret.getConfig().getZeroOffsetFromRobotFront())
                                    .plus(Rotation2d.fromDegrees(turret.getPositionDegrees()));
                    double launchElevationDegrees = 90 - params.hoodAngle();
                    Rotation3d launchRotation =
                            new Rotation3d(
                                    0,
                                    -Math.toRadians(launchElevationDegrees),
                                    turretFieldAngle.getRadians());
                    Translation3d launchVelocity =
                            new Translation3d(params.exitSpeedMs(), launchRotation);

                    Transform2d robotToTurret =
                            new Transform2d(
                                    new Translation2d(
                                            TURRET_PIVOT_POINT.getX(), TURRET_PIVOT_POINT.getY()),
                                    Rotation2d.kZero);
                    Translation3d launcherPose =
                            new Translation3d(robotPos.transformBy(robotToTurret).getTranslation())
                                    .plus(
                                            new Translation3d(
                                                    0,
                                                    0,
                                                    Units.inchesToMeters(LAUNCH_HEIGHT_INCHES)));
                    ballSim.launchBall(launcherPose, launchVelocity, 500);
                });
    }

    public Command ballSimLaunchFuel() {
        if (!Utils.isSimulation()) {
            return Commands.none();
        }
        return Commands.defer(
                () -> {
                    int fuelCount = ballSim.getTotalIntaked();
                    SequentialCommandGroup stream =
                            new SequentialCommandGroup(Commands.waitSeconds(Math.random() * 0.3));
                    for (int i = 0; i < fuelCount; i++) {
                        stream.addCommands(
                                createSimBallLaunch(),
                                Commands.waitSeconds(timeBetweenBallLaunches));
                    }
                    return stream.withName("RobotSim.ballSimLaunchFuel");
                },
                Set.of() // no subsystem requirements
                );
    }
}
