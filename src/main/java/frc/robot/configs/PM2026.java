package frc.robot.configs;

import frc.robot.Robot.Config;

public class PM2026 extends Config {

    // Final Machine
    public PM2026() {
        super();
        swerve.configEncoderOffsets(
                -0.312744140625 + 0.5, -0.032470703125 + 0.5, 0.3544921875 - 0.5, -0.4765625 + 0.5);
        turret.setCANcoderOffset(0.132568359375);

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
