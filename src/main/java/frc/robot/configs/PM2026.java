package frc.robot.configs;

import frc.robot.Robot.Config;

public class PM2026 extends Config {

    public PM2026() {
        super();
        // CANcoder offsets in rotations.
        swerve.configEncoderOffsets(
                -0.312744140625 + 0.5, -0.032470703125 + 0.5, 0.3544921875 - 0.5, -0.4765625 + 0.5);

        pilot.setAttached(true);
        operator.setAttached(true);
        intakeRoller.setAttached(true);
        intakeKicker.setAttached(true);
        intakeExtensionLeft.setAttached(true);
        intakeExtensionRight.setAttached(true);
        launcher.setAttached(true);
    }
}
