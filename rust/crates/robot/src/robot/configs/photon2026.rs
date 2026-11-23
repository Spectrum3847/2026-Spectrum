use crate::robot::robot::Config;

/// Photon Machine.
pub fn photon2026() -> Config {
    let mut config = Config::default();
    config
        .swerve
        .config_encoder_offsets(0.127197265625, -0.260009765625, 0.1171875, -0.3427734375);

    config.pilot.base.attached = true;
    config.operator.base.attached = true;
    config.fuel_intake.base.attached = true;
    config.intake_extension.base.attached = true;
    config.launcher.base.attached = true;
    config.indexer_tower.base.attached = true;
    config.indexer_bed.base.attached = true;
    config
}
