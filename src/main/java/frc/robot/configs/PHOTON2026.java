package frc.robot.configs;

import frc.robot.Robot.Config;

public class PHOTON2026 extends Config {

    // Photon Machine
    public PHOTON2026() {
        super();
        swerve.configEncoderOffsets(0.122803, -0.247803, 0.116455, -0.359619);

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
