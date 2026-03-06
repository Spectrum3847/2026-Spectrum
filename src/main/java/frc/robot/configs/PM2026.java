package frc.robot.configs;

import frc.robot.Robot.Config;

public class PM2026 extends Config {

    // Final Machine
    public PM2026() {
        super();
        swerve.configEncoderOffsets(-0.312744140625, -0.032470703125, 0.3544921875, -0.4765625);
        turret.setCANcoderOffset(0.12158203125);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        fuelIntake.setAttached(false);
        intakeExtension.setAttached(false);
        turret.setAttached(true);
        launcher.setAttached(false);
        indexerTower.setAttached(false);
        indexerBed.setAttached(false);
    }
}
