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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.rebuilt.FuelPhysicsSim;
import frc.rebuilt.ShotCalculator;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.CurrentSuperState;
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

    public static Trigger simLaunching() {
        return new Trigger(
                () ->
                        Utils.isSimulation()
                                && (Robot.getSuperStructure().getCurrentSuperState()
                                                == CurrentSuperState.LAUNCH_WITH_SQUEEZE
                                        || Robot.getSuperStructure().getCurrentSuperState()
                                                == CurrentSuperState.LAUNCH_WITHOUT_SQUEEZE
                                        || Robot.getSuperStructure().getCurrentSuperState()
                                                == CurrentSuperState
                                                        .LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY));
    }

    @Getter private static double simRobotWidth = Units.inchesToMeters(33);
    @Getter private static double simRobotLength = Units.inchesToMeters(32.75);

    private static final Translation3d SHOOTER_TURRET_PIVOT_POINT =
            new Translation3d(-Units.inchesToMeters(11.00), 0, Units.inchesToMeters(19.237));

    @Getter private FuelPhysicsSim ballSim;
    private int singleLaneBPS = 8;
    private double timeBetweenBallLaunches = 1.0 / singleLaneBPS;
    private double launcherWidth = 24;
    private int numOfLanes = 4;
    private double laneWidth = launcherWidth / numOfLanes;
    private double lane1 = -2 * laneWidth / 2;
    private double lane2 = -1 * laneWidth / 2;
    private double lane3 = 1 * laneWidth / 2;
    private double lane4 = 2 * laneWidth / 2;

    private SuperStructure robotSuperStructure;

    public RobotSim(SuperStructure superStructure) {
        this.robotSuperStructure = superStructure;
        SmartDashboard.putData("Sim/LeftView", RobotSim.leftView);
        leftView.setBackgroundColor(new Color8Bit(Color.kLightGray));
        drawRobot();

        ballSim = new FuelPhysicsSim("Sim/Fuel");
        ballSim.enable();
        ballSim.placeFieldBalls(); // spawns all the game pieces
        configBallSimRobot();
    }

    public void updateArticulatedMechanisms() {
        double intakeExtensionPose =
                Units.inchesToMeters(12)
                        * robotSuperStructure.getIntakeExtension().getPositionPercentage()
                        / 100;
        var intakePose3d =
                Pose3d.kZero.plus(
                        new Transform3d(
                                new Translation3d(intakeExtensionPose, 0, 0), Rotation3d.kZero));

        double turretAngleDegrees = robotSuperStructure.getTurret().getPositionDegrees();
        var turretPose3d =
                Pose3d.kZero.rotateAround(
                        SHOOTER_TURRET_PIVOT_POINT,
                        new Rotation3d(0, -Math.toRadians(turretAngleDegrees - 9), 0));

        Pose3d[] mechanismPoses = {intakePose3d, turretPose3d};

        Telemetry.log("Sim/Components", mechanismPoses);
    }

    public void drawRobot() {
        drawSideRobot();
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
    }

    private void configBallSimRobot() {
        double bumperHeight = Units.inchesToMeters(4.56);
        double intakeWidth = Units.inchesToMeters(10);
        double intakeLength = Units.inchesToMeters(28.5);
        double intakeXMin = Units.inchesToMeters(0);
        double intakeXMax = simRobotWidth / 2 + intakeWidth;
        double intakeYMin = -intakeLength / 2;
        double intakeYMax = intakeLength / 2;
        ballSim.configureRobot(
                simRobotWidth,
                simRobotLength,
                bumperHeight,
                50,
                () -> Robot.getSwerve().getRobotPose(),
                () -> Robot.getSwerve().getCurrentRobotChassisSpeeds());
        ballSim.addIntakeZone(
                intakeXMin,
                intakeXMax,
                intakeYMin,
                intakeYMax,
                () ->
                        Robot.getSuperStructure().getCurrentSuperState()
                                == CurrentSuperState.INTAKE_FUEL);
    }

    private Command createSimBallLaunch(double laneOffset) {
        return Commands.runOnce(
                () -> {
                    var params = ShotCalculator.getInstance().getParameters();
                    double launchSpeed = params.flywheelSpeed();
                    double launchAngle = Math.toRadians(90 - params.turretAngle().getDegrees());
                    double launchYaw =
                            Robot.getSwerve().getRobotPose().getRotation().getRadians()
                                    + Math.toRadians(180);
                    Rotation3d launchRotation = new Rotation3d(0, -launchAngle, launchYaw);
                    Translation3d launchVelocity = new Translation3d(launchSpeed, launchRotation);

                    Pose2d robotPos = Robot.getSwerve().getRobotPose();
                    Transform2d robotToLauncher =
                            new Transform2d(
                                    new Translation2d(
                                            Units.inchesToMeters(-5.5),
                                            Units.inchesToMeters(laneOffset)),
                                    Rotation2d.k180deg);
                    Translation3d launcherPose =
                            new Translation3d(
                                            robotPos.transformBy(robotToLauncher).getTranslation())
                                    .plus(new Translation3d(0, 0, Units.inchesToMeters(17.5)));
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
                    int numToLaunchPerLane = fuelCount / numOfLanes;
                    SequentialCommandGroup group1 =
                            new SequentialCommandGroup(Commands.waitSeconds(Math.random() * 0.3));
                    for (int i = 0; i < numToLaunchPerLane; i++) {
                        group1.addCommands(
                                createSimBallLaunch(lane1),
                                Commands.waitSeconds(timeBetweenBallLaunches));
                    }
                    SequentialCommandGroup group2 =
                            new SequentialCommandGroup(Commands.waitSeconds(Math.random() * 0.3));
                    for (int i = 0; i < numToLaunchPerLane; i++) {
                        group2.addCommands(
                                createSimBallLaunch(lane2),
                                Commands.waitSeconds(timeBetweenBallLaunches));
                    }
                    SequentialCommandGroup group3 =
                            new SequentialCommandGroup(Commands.waitSeconds(Math.random() * 0.3));
                    for (int i = 0; i < numToLaunchPerLane; i++) {
                        group3.addCommands(
                                createSimBallLaunch(lane3),
                                Commands.waitSeconds(timeBetweenBallLaunches));
                    }
                    SequentialCommandGroup group4 =
                            new SequentialCommandGroup(Commands.waitSeconds(Math.random() * 0.3));
                    for (int i = 0; i < numToLaunchPerLane; i++) {
                        group4.addCommands(
                                createSimBallLaunch(lane4),
                                Commands.waitSeconds(timeBetweenBallLaunches));
                    }
                    return Commands.parallel(group1, group2, group3, group4)
                            .withName("RobotSim.ballSimLaunchFuel");
                },
                Set.of() // no subsystem requirements
                );
    }
}
