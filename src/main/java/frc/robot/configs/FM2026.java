package frc.robot.configs;

import frc.robot.Robot.Config;

public class FM2026 extends Config {

    // Final Machine
    public FM2026() {
        super();
        swerve.configEncoderOffsets(-0.160400390625, 0.2392578125, 0.2744140625, -0.311767578125);

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
