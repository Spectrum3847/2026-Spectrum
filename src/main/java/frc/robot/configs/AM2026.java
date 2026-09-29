package frc.robot.configs;

import frc.robot.Robot.Config;

public class AM2026 extends Config {

    public AM2026() {
        super();
        // CANcoder offsets in rotations.
        swerve.configEncoderOffsets(0.289551, 0.394043, -0.203857, -0.039307);

        pilot.setAttached(true);
        operator.setAttached(true);
        intakeRoller.setAttached(false);
        intakeKicker.setAttached(false);
        intakeExtensionLeft.setAttached(false);
        intakeExtensionRight.setAttached(false);
        launcher.setAttached(false);
    }
}
