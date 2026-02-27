package frc.robot.configs;

import frc.robot.Robot.Config;

public class XM2026 extends Config {

    // Experimental Machine
    public XM2026() {
        super();
        swerve.configEncoderOffsets(
                0.1892089844 - 0.5,
                -0.2736816406 + 0.5,
                -0.404052734375 + 0.5,
                -0.478759765625 + 0.5);
        turret.setCANcoderOffset(-0.196533203125);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        fuelIntake.setAttached(true);
        intakeExtension.setAttached(false);
        turret.setAttached(true);
        launcher.setAttached(true);
        indexerTower.setAttached(true);
        indexerBed.setAttached(true);
    }
}
