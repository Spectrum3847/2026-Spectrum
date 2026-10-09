import { LogWriter } from "./make-log.mjs";

/**
 * A synthetic turret log with known faults injected, so the analyses can be tested against
 * numbers we chose rather than numbers we hope are right.
 *
 * Timeline (seconds):
 *   0-5     disabled
 *   5-60    enabled, turret tracking a slowly moving target, zero error steady at BASELINE
 *   25.0    MECHANICAL SLIP of SLIP_DEG: the camera's zero error steps and stays stepped, while
 *           the encoder sees nothing. The controller cannot correct this.
 *   40.0    ENCODER JUMP of JUMP_DEG: position reads a sudden offset, the controller drives it
 *           back out over ~0.6 s. The camera is unaffected.
 *   50-52   an unwrap: a legitimate 360 degree slew that must NOT be counted as a fault.
 */
export const BASELINE_DEG = -1.2;
export const SLIP_DEG = 7.5;
export const SLIP_T = 25.0;
export const JUMP_DEG = 14;
export const JUMP_T = 40.0;
export const UNWRAP_FROM = 50.0;
export const UNWRAP_TO = 52.0;
export const TOLERANCE_DEG = 2;

export function buildTurretLog() {
    const w = new LogWriter();
    const HZ = 50;
    const END = 60;

    w.put("DS:enabled", "boolean", 0, false);
    w.put("DS:enabled", "boolean", 5, true);
    w.put("DS:enabled", "boolean", END, false);

    // Where the turret is actually told to go: a slow sweep.
    const target = (t) => 20 * Math.sin((t - 5) / 9);

    // Encoder reading. Follows the command, plus the encoder jump, plus the unwrap slew.
    const encoder = (t) => {
        let p = target(t);
        if (t >= UNWRAP_FROM && t < UNWRAP_TO) p -= 360 * ((t - UNWRAP_FROM) / (UNWRAP_TO - UNWRAP_FROM));
        else if (t >= UNWRAP_TO) p = target(t);
        // The encoder jump appears instantly and is then driven out over 0.6 s.
        if (t >= JUMP_T && t < JUMP_T + 0.6) p += JUMP_DEG * (1 - (t - JUMP_T) / 0.6);
        return p;
    };

    w.series("/Robot/Turret/CommandedDegrees", "double", { from: 0, to: END, hz: HZ, fn: target });
    w.series("/Robot/Turret/PositionDegrees", "double", { from: 0, to: END, hz: HZ, fn: encoder });
    w.series("/Robot/Turret/PositionError", "double", { from: 0, to: END, hz: HZ, fn: (t) => target(t) - encoder(t) });
    w.series("/Robot/Turret/StatorCurrent", "double", { from: 0, to: END, hz: HZ, fn: (t) => (t < 5 ? 0 : Math.min(80, 6 + (30 * Math.abs(target(t) - encoder(t))) / 10)) });
    w.series("/Robot/Turret/Voltage", "double", { from: 0, to: END, hz: HZ, fn: (t) => (t < 5 ? 0 : Math.max(-6, Math.min(6, (target(t) - encoder(t)) * 0.8))) });
    w.series("/Robot/Turret/Unwrapping", "boolean", { from: 0, to: END, hz: 10, fn: (t) => t >= UNWRAP_FROM && t < UNWRAP_TO });
    w.series("/Robot/Turret/ReadyToShoot", "boolean", { from: 0, to: END, hz: 10, fn: (t) => t > 5 && Math.abs(target(t) - encoder(t)) <= TOLERANCE_DEG && !(t >= UNWRAP_FROM && t < UNWRAP_TO) });
    w.series("/Robot/Turret/SystemState", "string", { from: 0, to: END, hz: 2, fn: (t) => (t < 5 ? "IDLE" : "AIM_AT_TARGET") });

    // The camera's independent view. Steady, then permanently stepped by the mechanical slip.
    // Deliberately unaffected by the encoder jump -- the mechanism never moved for that one.
    w.series("/Robot/Vision/TurretLL/HeadingErrorDeg", "double", {
        from: 5, to: END, hz: 10,
        fn: (t) => BASELINE_DEG + (t >= SLIP_T ? SLIP_DEG : 0) + 0.15 * Math.sin(t * 3),
    });
    w.series("/Robot/Vision/TurretLL/IntegratedThisLoop", "boolean", { from: 5, to: END, hz: 10, fn: (t) => Math.floor(t * 10) % 4 === 0 });
    w.series("/Robot/Vision/BackLeftLL/IntegratedThisLoop", "boolean", { from: 5, to: END, hz: 10, fn: (t) => Math.floor(t * 10) % 3 === 0 });
    w.series("/Robot/ShotCalc/TurretAngleDeg", "double", { from: 5, to: END, hz: HZ, fn: (t) => target(t) + 0.4 * Math.sin(t * 7) });

    return w.buffer();
}
