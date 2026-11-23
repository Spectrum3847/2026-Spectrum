use crate::robot::robot::Config;

/// Experimental Machine. Nothing selects it, as in Java.
pub fn xm2026() -> Config {
    let mut config = Config::default();
    config.swerve.config_encoder_offsets(
        0.1892089844 - 0.5,
        -0.2736816406 + 0.5,
        -0.404052734375 + 0.5,
        -0.478759765625 + 0.5,
    );

    config.pilot.base.attached = true;
    config.operator.base.attached = true;
    config.fuel_intake.base.attached = true;
    config.intake_extension.base.attached = false;
    config.launcher.base.attached = true;
    config.indexer_tower.base.attached = true;
    config.indexer_bed.base.attached = true;
    config
}
