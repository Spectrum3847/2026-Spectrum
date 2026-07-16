package frc.robot.configs;

import frc.robot.Robot.Config;

public class OM2026 extends Config {
    public OM2026() {
        super();

        // TODO: change when robot is built and can be updated
        swerve.configEncoderOffsets(0, 0, 0, 0);

        pilot.setAttached(true);
        operator.setAttached(true);
        fuelIntake.setAttached(true);
        intakeExtension.setAttached(true);
        launcher.setAttached(true);
        spindexer.setAttached(true);
        turret.setAttached(true);
    }
}
