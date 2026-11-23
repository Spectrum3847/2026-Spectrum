use crate::robot::robot::Config;

/// Alpha Machine.
pub fn am2026() -> Config {
    let mut config = Config::default();
    config
        .swerve
        .config_encoder_offsets(0.289551, 0.394043, -0.203857, -0.039307);

    config.pilot.base.attached = true;
    config.operator.base.attached = true;
    config.fuel_intake.base.attached = false;
    config.intake_extension.base.attached = false;
    config.indexer_tower.base.attached = false;
    config.launcher.base.attached = false;
    config
}
