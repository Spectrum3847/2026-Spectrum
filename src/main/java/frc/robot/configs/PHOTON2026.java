package frc.robot.configs;

import frc.robot.Robot.Config;

public class PHOTON2026 extends Config {

    // Photon Machine
    public PHOTON2026() {
        super();
        swerve.configEncoderOffsets(0.127197265625, -0.260009765625, 0.1171875, -0.3427734375);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        fuelIntake.setAttached(true);
        intakeExtension.setAttached(true);
        launcher.setAttached(true);
        indexerTower.setAttached(true);
        indexerBed.setAttached(true);
    }
}
