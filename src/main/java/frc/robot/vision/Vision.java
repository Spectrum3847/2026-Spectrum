package frc.robot.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.rebuilt.FieldHelpers;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.Telemetry.PrintPriority;
import frc.spectrumLib.util.Util;
import frc.spectrumLib.vision.Limelight;
import frc.spectrumLib.vision.Limelight.LimelightConfig;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class Vision implements NTSendable, Subsystem {

    public static class VisionConfig {
        @Getter final String name = "Vision";

        /* Limelight Configuration */
        @Getter final String frontLL = "limelight-front";
        @Getter final LimelightConfig frontConfig = new LimelightConfig(frontLL)
                .withTranslation(0.215, 0, 0.188)
                .withRotation(0, Math.toRadians(28), 0);

        @Getter final String backLL = "limelight-back";
        @Getter final LimelightConfig backConfig = new LimelightConfig(backLL)
                .withTranslation(-0.215, 0.0, 0.188)
                .withRotation(0, Math.toRadians(28), Math.toRadians(180));

        @Getter final String leftLL = "limelight-left";
        @Getter final LimelightConfig leftConfig = new LimelightConfig(leftLL)
                .withTranslation(0, 0.215, 0.188)
                .withRotation(0, Math.toRadians(28), Math.toRadians(90));

        @Getter final String rightLL = "limelight-right";
        @Getter final LimelightConfig rightConfig = new LimelightConfig(rightLL)
                .withTranslation(0, -0.215, 0.188)
                .withRotation(0, Math.toRadians(28), Math.toRadians(-90));

        // Turret must be ZEROED in LL-GUI to report a correct pose
        // Dynamic translation and rotation will be applied in code based on turret and robot rotation
        @Getter final String turretLL = "limelight-turret";
        @Getter final LimelightConfig turretConfig = new LimelightConfig(turretLL)
                .withTranslation(
                    Units.inchesToMeters(-0.467), 
                    Units.inchesToMeters(-5.5), 
                    Units.inchesToMeters(27))
                .withRotation(
                    0, 
                    Math.toRadians(10), 
                    0);

        @Getter final Translation2d robotToTurretCenter = 
            new Translation2d(
                Units.inchesToMeters(-5.0),
                Units.inchesToMeters(-5.5));

        /* Pipeline configs */
        @Getter final int frontTagPipeline = 0;
        @Getter final int backTagPipeline = 0;
        @Getter final int leftTagPipeline = 0;
        @Getter final int rightTagPipeline = 0;
        @Getter final int turretTagPipeline = 0;

        /* Pose Estimation Constants */
        @Getter double visionStdDevX = 0.5;
        @Getter double visionStdDevY = 0.5;
        @Getter double visionStdDevTheta = 0.2;

        @Getter final double kLargeVariance = 999999.0; // Don't fuse rotation if variance exceeds this
        @Getter final double kMaxTimeDeltaSeconds = 0.1; // Max time difference to consider fusion

        @Getter final Matrix<N3, N1> visionStdMatrix = VecBuilder.fill(visionStdDevX, visionStdDevY, visionStdDevTheta);
    }

    /* Limelights */
    @Getter public final Limelight frontLL;
    @Getter public final Limelight backLL;
    @Getter public final Limelight leftLL;
    @Getter public final Limelight rightLL;
    @Getter public final Limelight turretLL;

    public final Limelight[] allLimelights;
    public final Limelight[] swerveLimelights; // Non-turret cameras

    private final DecimalFormat df = new DecimalFormat();

    @Getter private boolean isAiming = false;
    @Getter private DoubleSupplier turretRotationSupplier = () -> Robot.getTurret().getPositionDegrees();

    int[] blueTags = { 18, 19, 20, 21, 24, 25, 26, 27 };
    int[] redTags = { 2, 3, 4, 5, 8, 9, 10, 11, 12 };

    @Getter private static AprilTagFieldLayout tagLayout;

    private VisionConfig config;

    public Vision(VisionConfig config) {
        this.config = config;

        frontLL = new Limelight(config.frontLL, config.frontTagPipeline, config.frontConfig);
        backLL = new Limelight(config.backLL, config.backTagPipeline, config.backConfig);
        leftLL = new Limelight(config.leftLL, config.leftTagPipeline, config.leftConfig);
        rightLL = new Limelight(config.rightLL, config.rightTagPipeline, config.rightConfig);
        turretLL = new Limelight(config.turretLL, config.turretTagPipeline, config.turretConfig);

        swerveLimelights = new Limelight[] { frontLL, backLL, leftLL, rightLL };
        allLimelights = new Limelight[] { frontLL, backLL, leftLL, rightLL, turretLL };

        // logging
        df.setMaximumFractionDigits(2);

        /* Configure Limelight Settings Here */
        for (Limelight limelight : swerveLimelights) {
            limelight.setLEDMode(false);
            limelight.setIMUmode(1);
        }

        turretLL.setLEDMode(false);
        turretLL.setIMUmode(2); // Different IMU mode for turret

        tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        this.register();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public String getName() {
        return config.getName();
    }

    // Setup the telemetry values, has to be called at the end of the implemented mechanism constructor
    public void telemetryInit() {
        SendableRegistry.add(this, getName());
        SmartDashboard.putData(this);

        Robot.getField2d().getObject(frontLL.getCameraName());
        Robot.getField2d().getObject(backLL.getCameraName());
        Robot.getField2d().getObject(leftLL.getCameraName());
        Robot.getField2d().getObject(rightLL.getCameraName());
        Robot.getField2d().getObject(turretLL.getCameraName());
    }

    @Override
    public void periodic() {
        setLimeLightOrientation();
        disabledLimelightUpdates();
        enabledLimelightUpdates();

        Robot.getField2d().getObject(frontLL.getCameraName()).setPose(getFrontMegaTag2Pose());
        Robot.getField2d().getObject(backLL.getCameraName()).setPose(getBackMegaTag2Pose());
        Robot.getField2d().getObject(leftLL.getCameraName()).setPose(getLeftMegaTag2Pose());
        Robot.getField2d().getObject(rightLL.getCameraName()).setPose(getRightMegaTag2Pose());
        Robot.getField2d().getObject(turretLL.getCameraName()).setPose(getTurretMegaTag1Pose());
    }

    public Pose2d getFrontMegaTag2Pose() {
        Pose2d pose = frontLL.getMegaTag2_Pose2d();
        if (pose != null) {
            return pose;
        }
        return new Pose2d();
    }

    public Pose2d getBackMegaTag2Pose() {
        Pose2d pose = backLL.getMegaTag2_Pose2d();
        if (pose != null) {
            return pose;
        }
        return new Pose2d();
    }

    public Pose2d getLeftMegaTag2Pose() {
        Pose2d pose = leftLL.getMegaTag2_Pose2d();
        if (pose != null) {
            return pose;
        }
        return new Pose2d();
    }

    public Pose2d getRightMegaTag2Pose() {
        Pose2d pose = rightLL.getMegaTag2_Pose2d();
        if (pose != null) {
            return pose;
        }
        return new Pose2d();
    }

    public Pose2d getTurretMegaTag2Pose() {
        Pose2d pose = turretLL.getMegaTag2_Pose2d();
        if (pose != null) {
            return pose;
        }
        return new Pose2d();
    }

    public Pose2d getTurretMegaTag1Pose() {
        Pose2d pose = turretLL.getMegaTag1_Pose3d().toPose2d();
        if (pose != null) {
            return pose;
        }
        return new Pose2d();
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.addDoubleProperty("FrontTX", frontLL::getTagTx, null);
        builder.addDoubleProperty("FrontTA", frontLL::getTagTA, null);
        builder.addDoubleProperty("FrontTagID", frontLL::getClosestTagID, null);
        builder.addDoubleProperty("BackTX", backLL::getTagTx, null);
        builder.addDoubleProperty("BackTA", backLL::getTagTA, null);
        builder.addDoubleProperty("BackTagID", backLL::getClosestTagID, null);
        builder.addDoubleProperty("LeftTX", leftLL::getTagTx, null);
        builder.addDoubleProperty("LeftTA", leftLL::getTagTA, null);
        builder.addDoubleProperty("LeftTagID", leftLL::getClosestTagID, null);
        builder.addDoubleProperty("RightTX", rightLL::getTagTx, null);
        builder.addDoubleProperty("RightTA", rightLL::getTagTA, null);
        builder.addDoubleProperty("RightTagID", rightLL::getClosestTagID, null);
        builder.addDoubleProperty("TurretTX", turretLL::getTagTx, null);
        builder.addDoubleProperty("TurretTA", turretLL::getTagTA, null);
        builder.addDoubleProperty("TurretTagID", turretLL::getClosestTagID, null);
    }

    private void setLimeLightOrientation() {
        double yaw = Robot.getSwerve().getRobotPose().getRotation().getDegrees();

        for (Limelight limelight : allLimelights) {
            limelight.setRobotOrientation(yaw);
        }
    }

    private void disabledLimelightUpdates() {
        if (Util.disabled.getAsBoolean()) {
            for (Limelight limelight : allLimelights) {
                limelight.setIMUmode(1);
            }
            // MegaTag1 estimates (3D) for each swerve camera
            VisionFieldPoseEstimate frontMT1 = getMT1VisionEstimate(frontLL, true);
            VisionFieldPoseEstimate backMT1 = getMT1VisionEstimate(backLL, true);
            VisionFieldPoseEstimate leftMT1 = getMT1VisionEstimate(leftLL, true);
            VisionFieldPoseEstimate rightMT1 = getMT1VisionEstimate(rightLL, true);
            integrateMultipleEstimates(frontMT1, backMT1, leftMT1, rightMT1);

            // Turret estimate
            if (Robot.getTurret().isAttached()) {
                VisionFieldPoseEstimate turretMT1 = getMT1TurretEstimate(turretLL, true, true);
                integrateSingleEstimate(turretMT1);
            }
        }
    }

    private void enabledLimelightUpdates() {
        if (Util.teleop.getAsBoolean()) {
            for (Limelight limelight : allLimelights) {
                limelight.setIMUmode(4);
            }
            // MegaTag1 estimates (3D) for each swerve camera
            VisionFieldPoseEstimate frontMT1 = getMT1VisionEstimate(frontLL, false);
            VisionFieldPoseEstimate backMT1 = getMT1VisionEstimate(backLL, false);
            VisionFieldPoseEstimate leftMT1 = getMT1VisionEstimate(leftLL, false);
            VisionFieldPoseEstimate rightMT1 = getMT1VisionEstimate(rightLL, false);
            integrateMultipleEstimates(frontMT1, backMT1, leftMT1, rightMT1);

            // MegaTag2 estimates (2D) for each swerve camera
            VisionFieldPoseEstimate frontMT2 = getMT2VisionEstimate(frontLL);
            VisionFieldPoseEstimate backMT2 = getMT2VisionEstimate(backLL);
            VisionFieldPoseEstimate leftMT2 = getMT2VisionEstimate(leftLL);
            VisionFieldPoseEstimate rightMT2 = getMT2VisionEstimate(rightLL);
            integrateMultipleEstimates(frontMT2, backMT2, leftMT2, rightMT2);

            // Turret estimate
            if (Robot.getTurret().isAttached()) {
                VisionFieldPoseEstimate turretMT1 = getMT1TurretEstimate(turretLL, true, false);
                integrateSingleEstimate(turretMT1);
            }
        }
    }

    private VisionFieldPoseEstimate getMT1VisionEstimate(Limelight ll, boolean integrateXY) {
        if (!ll.targetInView()) {
            ll.setTagStatus("No Targets in View");
            ll.sendInvalidStatus("No Targets in View Rejection");
            return null;
        }

        boolean multiTags = ll.multipleTagsInView();
        double targetSize = ll.getTargetSize();
        Pose3d megaTag1Pose3d = ll.getMegaTag1_Pose3d();
        Pose2d megaTag1Pose2d = megaTag1Pose3d.toPose2d();
        RawFiducial[] tags = ll.getRawFiducial();
        double highestAmbiguity = 2;
        ChassisSpeeds robotSpeed = Robot.getSwerve().getCurrentRobotChassisSpeeds();
        double robotLinearSpeed = Math.hypot(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);

        // distance from current pose to vision estimated MT1 pose
        double mt1PoseDifference = Robot.getSwerve()
                .getRobotPose()
                .getTranslation()
                .getDistance(megaTag1Pose2d.getTranslation());

        // ambiguity / basic rejections
        ll.setTagStatus("");
        if (tags != null) {
            for (RawFiducial tag : tags) {
                if (highestAmbiguity == 2 || tag.ambiguity > highestAmbiguity) {
                    highestAmbiguity = tag.ambiguity;
                }
                if (tag.ambiguity > 0.9) {
                    // ambiguity too high -> reject
                    ll.sendInvalidStatus("High Ambiguity Rejection");
                    return null;
                }
            }
        }

        if (rejectionCheck(megaTag1Pose2d, targetSize)) {
            return null;
        }

        if (Math.abs(megaTag1Pose3d.getRotation().getX()) > 5
                || Math.abs(megaTag1Pose3d.getRotation().getY()) > 5) {
            // reject if pose is tilted in roll or pitch
            ll.sendInvalidStatus("Roll/Pitch Rejection");
            return null;
        }

        // Determine std devs similar to the original logic
        double xyStds;
        double degStds;

        if (robotLinearSpeed <= 0.2 && targetSize > 4) {
            ll.sendValidStatus("Stationary close integration");
            xyStds = 0.1;
            degStds = 0.1;
        } else if (multiTags && targetSize > 2) {
            ll.sendValidStatus("Strong Multi integration");
            xyStds = 0.1;
            degStds = 0.1;
        } else if (multiTags && targetSize > 0.2) {
            ll.sendValidStatus("Multi integration");
            xyStds = 0.25;
            degStds = 8;
        } else if (targetSize > 2 && (mt1PoseDifference < 0.5)) {
            ll.sendValidStatus("Close integration");
            xyStds = 0.5;
            degStds = config.getKLargeVariance();
        } else if (targetSize > 1 && (mt1PoseDifference < 0.25)) {
            ll.sendValidStatus("Proximity integration");
            xyStds = 1.0;
            degStds = config.getKLargeVariance();
        } else if (highestAmbiguity < 0.25 && targetSize >= 0.03) {
            ll.sendValidStatus("Stable integration");
            xyStds = 1.5;
            degStds = config.getKLargeVariance();
        } else {
            // shouldn't integrate
            ll.sendInvalidStatus("Integration Criteria not Met");
            return null;
        }

        // Strict with degree std and ambiguity for MegaTag1
        if (highestAmbiguity > 0.5) {
            degStds = 15;
        }

        if (robotSpeed.omegaRadiansPerSecond >= 0.5) {
            degStds = 50;
        }

        if (!integrateXY) {
            xyStds = 999999;
        }

        // If we're forcing integration (e.g., for testing), use very tight stds
        if (integrateXY) {
            xyStds = 0.01;
            degStds = 0.01;
        }

        Pose2d integratedPose = new Pose2d(megaTag1Pose2d.getTranslation(), megaTag1Pose2d.getRotation());

        double timestamp = Utils.fpgaToCurrentTime(ll.getMegaTag1PoseTimestamp());
        Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStds, xyStds, degStds);
        int numTags = tags == null ? 1 : tags.length;

        return new VisionFieldPoseEstimate(integratedPose, timestamp, stdDevs, numTags);
    }

    private VisionFieldPoseEstimate getMT2VisionEstimate(Limelight ll) {
        if (!ll.targetInView()) {
            ll.setTagStatus("No Targets in View");
            ll.sendInvalidStatus("No Targets in View Rejection");
            return null;
        }

        boolean multiTags = ll.multipleTagsInView();
        double targetSize = ll.getTargetSize();
        Pose2d megaTag2Pose2d = ll.getMegaTag2_Pose2d();
        ChassisSpeeds robotSpeed = Robot.getSwerve().getCurrentRobotChassisSpeeds();
        double robotLinearSpeed = Math.hypot(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);

        double mt2PoseDifference = Robot.getSwerve()
                .getRobotPose()
                .getTranslation()
                .getDistance(megaTag2Pose2d.getTranslation());

        /* rejections */
        if (rejectionCheck(megaTag2Pose2d, targetSize)) {
            return null;
        }

        /* Determine standard deviations */
        double xyStds;

        if (robotLinearSpeed <= 0.2 && targetSize > 4) {
            ll.sendValidStatus("Stationary close integration");
            xyStds = 0.1;
        } else if (multiTags && targetSize > 2) {
            ll.sendValidStatus("Strong Multi integration");
            xyStds = 0.1;
        } else if (multiTags && targetSize > 0.2) {
            ll.sendValidStatus("Multi integration");
            xyStds = 0.25;
        } else if (targetSize > 2 && (mt2PoseDifference < 0.5 || DriverStation.isDisabled())) {
            ll.sendValidStatus("Close integration");
            xyStds = 0.5;
        } else if (targetSize > 1 && (mt2PoseDifference < 0.25 || DriverStation.isDisabled())) {
            ll.sendValidStatus("Proximity integration");
            xyStds = 1.0;
        } else if (targetSize >= 0.03) {
            ll.sendValidStatus("Stable integration");
            xyStds = 1.5;
        } else {
            return null; // Shouldn't integrate
        }

        // MegaTag2 doesn't provide rotation, so use large variance
        double degStds = config.getKLargeVariance();

        Pose2d integratedPose = new Pose2d(
                megaTag2Pose2d.getTranslation(),
                megaTag2Pose2d.getRotation());

        return new VisionFieldPoseEstimate(
                integratedPose,
                Utils.fpgaToCurrentTime(ll.getMegaTag2PoseTimestamp()),
                VecBuilder.fill(xyStds, xyStds, degStds),
                (int) ll.getTagCountInView());
    }

    private VisionFieldPoseEstimate getMT1TurretEstimate(Limelight ll, boolean integrateXY, boolean forceIntegration) {
        if (!ll.targetInView()) {
            ll.setTagStatus("No Targets in View");
            ll.sendInvalidStatus("No Targets in View Rejection");
            return null;
        }

        boolean multiTags = ll.multipleTagsInView();
        double targetSize = ll.getTargetSize();
        Pose3d megaTag1Pose3d = ll.getMegaTag1_Pose3d();
        Pose2d megaTag1Pose2d = megaTag1Pose3d.toPose2d();
        RawFiducial[] tags = ll.getRawFiducial();

        double highestAmbiguity = 0.0;
        ChassisSpeeds robotSpeed = Robot.getSwerve().getCurrentRobotChassisSpeeds();
        double robotLinearSpeed = Math.hypot(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);

        double mt1PoseDifference = Robot.getSwerve()
                .getRobotPose()
                .getTranslation()
                .getDistance(megaTag1Pose2d.getTranslation());

        /* ---------------- Rejections ---------------- */
        ll.setTagStatus("");
        for (RawFiducial tag : tags) {
            highestAmbiguity = Math.max(highestAmbiguity, tag.ambiguity);

            if (tag.ambiguity > 0.9) {
                ll.sendInvalidStatus("High Ambiguity Rejection");
                return null;
            }
        }

        if (rejectionCheck(megaTag1Pose2d, targetSize)) {
            ll.sendInvalidStatus("Generic Rejection");
            return null;
        }

        // Roll / pitch rejection
        if (Math.abs(Math.toDegrees(megaTag1Pose3d.getRotation().getX())) > 5
                || Math.abs(Math.toDegrees(megaTag1Pose3d.getRotation().getY())) > 5) {
            ll.sendInvalidStatus("Roll/Pitch Rejection");
            return null;
        }

        /* ---------------- Integration tuning ---------------- */
        double xyStds;
        double degStds;

        if (robotLinearSpeed <= 0.2 && targetSize > 4) {
            ll.sendValidStatus("Stationary close integration");
            xyStds = 0.1;
            degStds = 0.1;
        } else if (multiTags && targetSize > 2) {
            ll.sendValidStatus("Strong multi integration");
            xyStds = 0.1;
            degStds = 0.1;
        } else if (multiTags && targetSize > 0.2) {
            ll.sendValidStatus("Multi integration");
            xyStds = 0.25;
            degStds = 8.0;
        } else if (targetSize > 2 && mt1PoseDifference < 0.5) {
            ll.sendValidStatus("Close integration");
            xyStds = 0.5;
            degStds = config.getKLargeVariance();
        } else if (targetSize > 1 && mt1PoseDifference < 0.25) {
            ll.sendValidStatus("Proximity integration");
            xyStds = 1.0;
            degStds = config.getKLargeVariance();
        } else if (highestAmbiguity < 0.25 && targetSize >= 0.03) {
            ll.sendValidStatus("Stable integration");
            xyStds = 1.5;
            degStds = config.getKLargeVariance();
        } else {
            ll.sendInvalidStatus("Confidence too low");
            return null;
        }

        /* ---------------- MT1-specific tightening ---------------- */
        // MT1 rotation is weak — trust it even less when ambiguity rises
        if (highestAmbiguity > 0.5) {
            degStds = Math.max(degStds, 50.0);
        }

        if (Math.abs(robotSpeed.omegaRadiansPerSecond) >= 0.5) {
            degStds = Math.max(degStds, 75.0);
        }

        if (!integrateXY) {
            xyStds = config.getKLargeVariance();
        }

        // If we're forcing integration, use very tight stds
        if (forceIntegration) {
            xyStds = 0.01;
            degStds = 0.01;
        }

        /* ---------------- Turret adjustment ---------------- */
        double turretDegrees = turretRotationSupplier.getAsDouble();
        Rotation2d turretRotation = Rotation2d.fromDegrees(turretDegrees);

        // Robot->Camera at 0° turret
        Translation2d robotToCamera0 = new Translation2d(
                config.getTurretConfig().getForward(),
                config.getTurretConfig().getRight() * -1); // Negate because left is positive in WPI coordinate system

        // Vector from turret center -> camera at zero turret
        Translation2d turretToCamera0 = robotToCamera0.minus(config.getRobotToTurretCenter());

        // Rotate vector by turret angle
        Translation2d turretToRotatedCamera = turretToCamera0.rotateBy(turretRotation);

        // Add turret center offset to get full robot->camera vector
        Translation2d robotToRotatedCamera = config.getRobotToTurretCenter().plus(turretToRotatedCamera);

        // Compute robot pose
        Translation2d robotTranslation = megaTag1Pose2d.getTranslation().minus(robotToRotatedCamera);
        Rotation2d robotRotation = megaTag1Pose2d.getRotation().minus(turretRotation);

        Pose2d integratedPose = new Pose2d(robotTranslation, robotRotation);

        double timestamp = Utils.fpgaToCurrentTime(ll.getMegaTag1PoseTimestamp());
        Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStds, xyStds, degStds);
        int numTags = tags == null ? 1 : tags.length;

        return new VisionFieldPoseEstimate(integratedPose, timestamp, stdDevs, numTags);
    }

    /** Helper to integrate a single estimate */
    private void integrateSingleEstimate(VisionFieldPoseEstimate estimate) {
        if (estimate != null) {
            Robot.getSwerve()
                    .addVisionMeasurement(
                            estimate.getVisionRobotPoseMeters(),
                            estimate.getTimestampSeconds(),
                            estimate.getVisionMeasurementStdDevs());
        }
    }

    /**
     * Helper to integrate multiple estimates close in time by fusing them together
     * first
     */
    private void integrateMultipleEstimates(VisionFieldPoseEstimate... estimates) {
        // Collect non-null estimates
        List<VisionFieldPoseEstimate> list = new ArrayList<>();
        for (VisionFieldPoseEstimate e : estimates) {
            if (e != null)
                list.add(e);
        }
        if (list.isEmpty())
            return;

        // Sort by timestamp ascending (old -> new). fuseEstimates expects to project
        // older -> newer.
        list.sort(Comparator.comparingDouble(VisionFieldPoseEstimate::getTimestampSeconds));

        // Iteratively group/fuse estimates close in time.
        VisionFieldPoseEstimate currentGroup = list.get(0);
        for (int i = 1; i < list.size(); i++) {
            VisionFieldPoseEstimate next = list.get(i);
            double timeDelta = Math.abs(next.getTimestampSeconds() - currentGroup.getTimestampSeconds());

            if (timeDelta < config.getKMaxTimeDeltaSeconds()) {
                // Fuse into current group (currentGroup older, next newer)
                currentGroup = fuseEstimates(currentGroup, next);
            } else {
                // No close timestamp: integrate current group and start a new one
                integrateSingleEstimate(currentGroup);
                currentGroup = next;
            }
        }
        // integrate the final fused group
        integrateSingleEstimate(currentGroup);
    }

    private boolean rejectionCheck(Pose2d pose, double targetSize) {
        /* rejections */
        if (FieldHelpers.poseOutOfField(pose)) {
            return true;
        }

        if (Math.abs(Robot.getSwerve().getCurrentRobotChassisSpeeds().omegaRadiansPerSecond) >= 1.6) {
            return true;
        }

        // Final check, if it's small reject, else return false and integrate
        return targetSize <= 0.025;
    }

    /**
     * Choose the limelight with the best view of multiple tags
     *
     * @return the best limelight
     */
    public Limelight getBestLimelight() {
        Limelight bestLimelight = frontLL;
        double bestScore = 0;
        for (Limelight limelight : allLimelights) {
            double score = 0;
            // prefer LL with most tags, when equal tag count, prefer LL closer to tags
            score += limelight.getTagCountInView();
            score += limelight.getTargetSize();

            if (score > bestScore) {
                bestScore = score;
                bestLimelight = limelight;
            }
        }
        return bestLimelight;
    }

    /** reset pose to the best limelight's vision pose */
    public void resetPoseToVision() {
        Limelight ll = getBestLimelight();
        resetPoseToVision(
                ll.targetInView(),
                ll.getMegaTag1_Pose3d(),
                ll.getMegaTag2_Pose2d(),
                ll.getMegaTag1PoseTimestamp());
    }

    /**
     * Set robot pose to vision pose only if LL has good tag reading
     *
     * @return if the pose was accepted and integrated
     */
    public boolean resetPoseToVision(
            boolean targetInView, Pose3d botpose3D, Pose2d megaPose, double poseTimestamp) {

        boolean reject = false;
        if (targetInView) {
            // replace botpose with this.pose
            Pose2d botpose = botpose3D.toPose2d();
            Pose2d pose;

            // Check if the vision pose is bad and don't trust it
            if (FieldHelpers.poseOutOfField(botpose3D)) { // pose out of field
                Telemetry.log("Pose out of field", reject);
                reject = true;
            } else if (Math.abs(botpose3D.getZ()) > 0.25) { // when in air
                Telemetry.log("Pose in air", reject);
                reject = true;
            } else if ((Math.abs(botpose3D.getRotation().getX()) > 5
                    || Math.abs(botpose3D.getRotation().getY()) > 5)) { // when tilted

                Telemetry.log("Pose tilted", reject);
                reject = true;
            }

            // don't continue
            if (reject) {
                return !reject; // return the success status
            }

            // Posts Current X,Y, and Angle (Theta) values
            double[] visionPose = {
                    botpose.getX(), botpose.getY(), botpose.getRotation().getDegrees()
            };
            Telemetry.log("Current Vision Pose: ", visionPose);

            Robot.getSwerve()
                    .setVisionMeasurementStdDevs(VecBuilder.fill(0.00001, 0.00001, 0.00001));

            Pose2d integratedPose = new Pose2d(megaPose.getTranslation(), botpose.getRotation());
            Robot.getSwerve().addVisionMeasurement(integratedPose, poseTimestamp);
            pose = Robot.getSwerve().getRobotPose();
            // Gets updated pose of x, y, and theta values
            visionPose = new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() };
            Telemetry.log("Vision Pose Reset To: ", visionPose);

            // print "success"
            return true;
        }
        return false; // target not in view
    }

    /**
     * If at least one LL has an accurate pose
     *
     * @return true if at least one LL has an accurate pose
     */
    public boolean hasAccuratePose() {
        for (Limelight limelight : allLimelights) {
            if (limelight.hasAccuratePose())
                return true;
        }
        return false;
    }

    /** Change all LL pipelines to the same pipeline */
    public void setLimelightPipelines(int pipeline) {
        for (Limelight limelight : allLimelights) {
            limelight.setLimelightPipeline(pipeline);
        }
    }

    public boolean isTurretSeeingTag() {
        return turretLL.targetInView() && turretLL.getTagTA() >= 2;
    }

    public boolean tagsInView() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
        if (alliance == DriverStation.Alliance.Blue) {
            return Arrays.stream(allLimelights)
                    .mapToInt(ll -> (int) ll.getClosestTagID())
                    .anyMatch(id -> Arrays.stream(blueTags).anyMatch(tag -> tag == id));
        } else if (alliance == DriverStation.Alliance.Red) {
            return Arrays.stream(allLimelights)
                    .mapToInt(ll -> (int) ll.getClosestTagID())
                    .anyMatch(id -> Arrays.stream(redTags).anyMatch(tag -> tag == id));
        } else {
            return false;
        }
    }

    // ------------------------------------------------------------------------------
    // VisionStates Commands
    // ------------------------------------------------------------------------------

    /** Set all Limelights to blink */
    public Command blinkLimelights() {
        Telemetry.print("Vision.blinkLimelights", PrintPriority.HIGH);
        return startEnd(
                () -> {
                    for (Limelight limelight : allLimelights) {
                        limelight.blinkLEDs();
                    }
                },
                () -> {
                    for (Limelight limelight : allLimelights) {
                        limelight.setLEDMode(false);
                    }
                })
                .withName("Vision.blinkLimelights");
    }

    /** Only blinks left limelight */
    public Command solidLimelight() {
        return startEnd(
                () -> {
                    for (Limelight limelight : allLimelights) {
                        limelight.setLEDMode(true);;
                    }
                },
                () -> {
                    for (Limelight limelight : allLimelights) {
                        limelight.setLEDMode(false);
                    }
                })
                .withName("Vision.solidLimelight");
    }

    /**
     * Fuses two vision pose estimates using inverse-variance weighting. (FRC254
     * 2025)
     */
    private VisionFieldPoseEstimate fuseEstimates(
            VisionFieldPoseEstimate a, VisionFieldPoseEstimate b) {
        // Ensure b is the newer measurement
        if (b.getTimestampSeconds() < a.getTimestampSeconds()) {
            VisionFieldPoseEstimate tmp = a;
            a = b;
            b = tmp;
        }

        // Project both estimates to the same timestamp using odometry
        Transform2d a_T_b = Robot.getSwerve().getPoseAtTimestamp(b.getTimestampSeconds())
                .minus(Robot.getSwerve().getPoseAtTimestamp(a.getTimestampSeconds()));

        Pose2d poseA = a.getVisionRobotPoseMeters().transformBy(a_T_b);
        Pose2d poseB = b.getVisionRobotPoseMeters();

        // Inverse‑variance weighting
        var varianceA = a.getVisionMeasurementStdDevs().elementTimes(a.getVisionMeasurementStdDevs());
        var varianceB = b.getVisionMeasurementStdDevs().elementTimes(b.getVisionMeasurementStdDevs());

        Rotation2d fusedHeading = poseB.getRotation();
        if (varianceA.get(2, 0) < config.getKLargeVariance()
                && varianceB.get(2, 0) < config.getKLargeVariance()) {
            fusedHeading = new Rotation2d(
                    poseA.getRotation().getCos() / varianceA.get(2, 0)
                            + poseB.getRotation().getCos() / varianceB.get(2, 0),
                    poseA.getRotation().getSin() / varianceA.get(2, 0)
                            + poseB.getRotation().getSin() / varianceB.get(2, 0));
        }

        double weightAx = 1.0 / varianceA.get(0, 0);
        double weightAy = 1.0 / varianceA.get(1, 0);
        double weightBx = 1.0 / varianceB.get(0, 0);
        double weightBy = 1.0 / varianceB.get(1, 0);

        Pose2d fusedPose = new Pose2d(
                new Translation2d(
                        (poseA.getTranslation().getX() * weightAx
                                + poseB.getTranslation().getX() * weightBx)
                                / (weightAx + weightBx),
                        (poseA.getTranslation().getY() * weightAy
                                + poseB.getTranslation().getY() * weightBy)
                                / (weightAy + weightBy)),
                fusedHeading);

        Matrix<N3, N1> fusedStdDev = VecBuilder.fill(
                Math.sqrt(1.0 / (weightAx + weightBx)),
                Math.sqrt(1.0 / (weightAy + weightBy)),
                Math.sqrt(1.0 / (1.0 / varianceA.get(2, 0) + 1.0 / varianceB.get(2, 0))));

        int numTags = a.getNumTags() + b.getNumTags();
        double time = b.getTimestampSeconds();

        return new VisionFieldPoseEstimate(fusedPose, time, fusedStdDev, numTags);
    }

    @Getter
    public class VisionFieldPoseEstimate {
        private final Pose2d visionRobotPoseMeters;
        private final double timestampSeconds;
        private final Matrix<N3, N1> visionMeasurementStdDevs;
        private final int numTags;

        public VisionFieldPoseEstimate(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs,
                int numTags) {
            this.visionRobotPoseMeters = visionRobotPoseMeters;
            this.timestampSeconds = timestampSeconds;
            this.visionMeasurementStdDevs = visionMeasurementStdDevs;
            this.numTags = numTags;
        }
    }
}