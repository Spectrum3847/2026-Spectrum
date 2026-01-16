package frc.robot.configs;

import frc.robot.Robot.Config;

public class AM2026 extends Config {

    // Alpha Machine
    public AM2026() {
        super();
        swerve.configEncoderOffsets(0.289551, 0.394043, -0.203857, -0.039307);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        intake.setAttached(true);
        turret.setAttached(false);
    }
}
