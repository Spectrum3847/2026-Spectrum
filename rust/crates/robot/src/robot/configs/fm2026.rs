use crate::robot::robot::Config;

/// Final Machine.
pub fn fm2026() -> Config {
    let mut config = Config::default();
    config
        .swerve
        .config_encoder_offsets(-0.163818359375, 0.24902, 0.2724609375, -0.31005859375);

    config.pilot.base.attached = true;
    config.operator.base.attached = true;
    config.fuel_intake.base.attached = true;
    config.intake_extension.base.attached = true;
    config.launcher.base.attached = true;
    config.indexer_tower.base.attached = true;
    config.indexer_bed.base.attached = true;
    config.hood.base.attached = true;
    config
}
