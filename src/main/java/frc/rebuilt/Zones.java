package frc.rebuilt;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.rebuilt.offsets.HomeOffsets;
import frc.robot.Robot;
import frc.robot.swerve.Swerve;

public class Zones {

    private static final Swerve swerve = Robot.getSwerve();
    private static final HomeOffsets offsets = new HomeOffsets();

    public static final Trigger blueFieldSide = swerve.inXzone(0, Field.getHalfLength());
    public static final Trigger opponentFieldSide =
            new Trigger(() -> blueFieldSide.getAsBoolean() != Field.isBlue());
}
