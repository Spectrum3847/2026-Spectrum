package frc.robot.configs;

import frc.robot.Robot.Config;

public class PHOTON2026 extends Config {

    // Photon Machine
    public PHOTON2026() {

        super();
        // swerve.configEnconderOffsets();

        // Attached Mechanisms
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
