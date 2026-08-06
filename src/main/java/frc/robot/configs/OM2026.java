package frc.robot.configs;

import frc.robot.Robot.Config;

public class OM2026 extends Config {
    /**
     * Initializes the 2026 Offseason robot configuration with attached subsystems and swerve
     * encoder offsets.
     */
    public OM2026() {
        super();

        swerve.configEncoderOffsets(
                -0.47216796875 + 0.5, -0.126220703125, -0.122802734375 + 0.5, -0.2744140625);

        pilot.setAttached(true);
        operator.setAttached(true);
        intakeRoller.setAttached(true);
        intakeKicker.setAttached(true);
        intakeExtensionLeft.setAttached(true);
        intakeExtensionRight.setAttached(true);
        launcher.setAttached(true);
        rotor.setAttached(true);
        feeder.setAttached(true);
        turret.setAttached(true);
        hood.setAttached(true);
    }
}
