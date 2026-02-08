package frc.robot.configs;

import frc.robot.Robot.Config;

public class XM2026 extends Config {

    // Experimental Machine
    public XM2026() {
        super();
        swerve.configEncoderOffsets(0.289551, 0.394043, -0.203857, -0.039307);
        turret.setCANcoderOffset(-0.196533203125);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        fuelIntake.setAttached(false);
        turret.setAttached(true);
        intakeExtension.setAttached(false);
        indexer.setAttached(false);
        launcher.setAttached(true);
    }
}
