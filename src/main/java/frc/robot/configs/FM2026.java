package frc.robot.configs;

import frc.robot.Robot.Config;

public class FM2026 extends Config {

    public FM2026() {
        super();
        // CANcoder offsets in rotations.
        swerve.configEncoderOffsets(-0.163818359375, 0.24902, 0.2724609375, -0.31005859375);

        pilot.setAttached(true);
        operator.setAttached(true);
        intakeRoller.setAttached(true);
        intakeKicker.setAttached(true);
        intakeExtensionLeft.setAttached(true);
        intakeExtensionRight.setAttached(true);
        launcher.setAttached(true);
        rotor.setAttached(true);
        feeder.setAttached(true);
    }
}
