package frc.robot.configs;

import frc.robot.Robot.Config;

public class FM2026 extends Config {

    // Final Machine
    public FM2026() {
        super();
        swerve.configEncoderOffsets(0.289551, 0.394043, -0.203857, -0.039307);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        // fuelIntake.setAttached(true);
        turret.setAttached(true);
        intakeExtension.setAttached(true);
    }
}
