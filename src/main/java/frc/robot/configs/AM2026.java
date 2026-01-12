package frc.robot.configs;

import frc.robot.Robot.Config;

public class AM2025 extends Config {

    // Alpha Robot

    public AM2025() {
        super();
        swerve.configEncoderOffsets(0.289551, 0.394043, -0.203857, -0.039307);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        intake.setAttached(false);
        turret.setAttached(false);
    }
}
