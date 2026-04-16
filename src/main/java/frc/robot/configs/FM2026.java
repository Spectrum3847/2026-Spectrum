package frc.robot.configs;

import frc.robot.Robot.Config;

public class FM2026 extends Config {

    // Final Machine
    public FM2026() {
        super();
        swerve.configEncoderOffsets(-0.1484375, 0.25439453125, 0.2666015625, -0.3017578125);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        fuelIntake.setAttached(true);
        intakeExtension.setAttached(true);
        launcher.setAttached(true);
        indexerTower.setAttached(true);
        indexerBed.setAttached(true);
        hood.setAttached(true);
    }
}
