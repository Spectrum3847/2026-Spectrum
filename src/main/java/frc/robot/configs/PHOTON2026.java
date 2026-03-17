package frc.robot.configs;

import frc.robot.Robot.Config;

public class PHOTON2026 extends Config {

    // Photon Machine
    public PHOTON2026() {
        super();
        
        // Configure Robot Hardware/Subsystems for Photon Team
        // TODO: Update offsets after physical assembly
        // swerve.configEnconderOffsets();
        // turret.setCANcoderOffset();

        // Attached Mechanisms
        // Enabling all subsystems as per team standard for testing
        pilot.setAttached(true);
        operator.setAttached(true);
        fuelIntake.setAttached(true);
        intakeExtension.setAttached(true);
        turret.setAttached(true);
        launcher.setAttached(true);
        indexerTower.setAttached(true);
        indexerBed.setAttached(true);
    }
}
