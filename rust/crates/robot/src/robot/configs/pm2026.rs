use crate::robot::robot::Config;

// Java: the class comment says "Final Machine", same as FM2026.
pub fn pm2026() -> Config {
    let mut config = Config::default();
    config.swerve.config_encoder_offsets(
        -0.312744140625 + 0.5,
        -0.032470703125 + 0.5,
        0.3544921875 - 0.5,
        -0.4765625 + 0.5,
    );

    config.pilot.base.attached = true;
    config.operator.base.attached = true;
    config.fuel_intake.base.attached = true;
    config.intake_extension.base.attached = true;
    config.launcher.base.attached = true;
    config.indexer_tower.base.attached = true;
    config.indexer_bed.base.attached = true;
    config
}
