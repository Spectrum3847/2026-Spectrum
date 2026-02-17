package frc.robot.configs;

import frc.robot.Robot.Config;

public class PHOTON2026 extends Config {

    public PHOTON2026() {

        super();
        // swerve.configEnconderOffsets();

        pilot.setAttached(true);
        operator.setAttached(true);
        fuelIntake.setAttached(true);
        turret.setAttached(true);
        intakeExtension.setAttached(true);
        indexer.setAttached(true);
        launcher.setAttached(true);

    }

}