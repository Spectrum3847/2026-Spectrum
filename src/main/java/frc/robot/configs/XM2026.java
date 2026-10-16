package frc.robot.configs;

import frc.robot.Robot.Config;

public class XM2026 extends Config {

    public XM2026() {
        super();
        // CANcoder offsets in rotations.
        swerve.configEncoderOffsets(
                0.1892089844 - 0.5,
                -0.2736816406 + 0.5,
                -0.404052734375 + 0.5,
                -0.478759765625 + 0.5);

        pilot.setAttached(true);
        operator.setAttached(true);
        intakeRoller.setAttached(true);
        intakeKicker.setAttached(true);
        intakeExtensionLeft.setAttached(false);
        intakeExtensionRight.setAttached(false);
        launcher.setAttached(true);
    }
}
