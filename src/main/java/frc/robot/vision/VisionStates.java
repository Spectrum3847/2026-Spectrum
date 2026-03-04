package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

public class VisionStates {

    private static Vision vision = Robot.getVision();

    public static final Trigger turretSeeingTag = new Trigger(vision::isTurretSeeingTag);
    public static final Trigger seeingTag = new Trigger(vision::tagsInView);

    public static void setupDefaultCommand() {
        vision.setDefaultCommand(vision.blinkLimelights().withName("Vision.default"));
    }

    public static void setStates() {}

    public static Command resetVisionPose() {
        return vision.runOnce(vision::resetPoseToVision)
                .withName("VisionStates.resetPoseToVision")
                .ignoringDisable(true);
    }

    public static Command blinkLimelights() {
        return vision.blinkLimelights().withName("VisionStates.blinkLimelights");
    }

    public static Command solidLimelight() {
        return vision.solidLimelight().withName("VisionCommands.solidLimelight");
    }
}
