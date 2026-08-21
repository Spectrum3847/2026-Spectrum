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
                -0.23046875 - 0.25, 0.10986328125 +0.25, -0.263916015625 + 0.125, 0.34716796875 - 0.125);

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
