package frc.robot.configs;

import frc.robot.Robot.Config;

public class PHOTON2026 extends Config {

    /**
     * Initializes the Photon Machine robot configuration, including swerve encoder offsets
     * and attached mechanisms.
     */
    public PHOTON2026() {
        super();
        swerve.configEncoderOffsets(0.127197265625, -0.260009765625, 0.1171875, -0.3427734375);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        intakeRoller.setAttached(true);
        intakeKicker.setAttached(true);
        intakeExtensionLeft.setAttached(true);
        intakeExtensionRight.setAttached(true);
        launcher.setAttached(true);
        // indexerTower.setAttached(true);
        // indexerBed.setAttached(true);
    }
}
