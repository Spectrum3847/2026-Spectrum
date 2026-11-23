// Port of 6328's ShiftHelpers (c) 2025-2026 Littleton Robotics, MIT license.

//! Hub shift clock: which alliance hub is active, and how long the current shift has run.
//!
//! The clock state is thread-local, like Java's statics, and belongs to the loop thread.
//! [`compute_shift_info`] and [`first_active_alliance_for`] are the pure parts.

use std::cell::{Cell, RefCell};
use telemetry::EnumName;
use wpilib::driver_station::{self, Alliance};
use wpilib::timer::Timer;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShiftEnum {
    Transition,
    Shift1,
    Shift2,
    Shift3,
    Shift4,
    Endgame,
    Auto,
    Disabled,
}

impl EnumName for ShiftEnum {
    fn name(&self) -> &'static str {
        match self {
            Self::Transition => "TRANSITION",
            Self::Shift1 => "SHIFT1",
            Self::Shift2 => "SHIFT2",
            Self::Shift3 => "SHIFT3",
            Self::Shift4 => "SHIFT4",
            Self::Endgame => "ENDGAME",
            Self::Auto => "AUTO",
            Self::Disabled => "DISABLED",
        }
    }
}

/// Enum constants in Java declaration order, indexed by shift number.
const SHIFTS_ENUMS: [ShiftEnum; 8] = [
    ShiftEnum::Transition,
    ShiftEnum::Shift1,
    ShiftEnum::Shift2,
    ShiftEnum::Shift3,
    ShiftEnum::Shift4,
    ShiftEnum::Endgame,
    ShiftEnum::Auto,
    ShiftEnum::Disabled,
];

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShiftInfo {
    pub current_shift: ShiftEnum,
    pub elapsed_time: f64,
    pub remaining_time: f64,
    pub active: bool,
}

const SHIFT_START_TIMES: [f64; 6] = [0.0, 10.0, 35.0, 60.0, 85.0, 110.0];
const SHIFT_END_TIMES: [f64; 6] = [10.0, 35.0, 60.0, 85.0, 110.0, 140.0];

const APPROACHING_ACTIVE_FUDGE: f64 = 2.0;
const ENDING_ACTIVE_FUDGE: f64 = 3.0;

pub const AUTO_END_TIME: f64 = 20.0;
pub const TELEOP_DURATION: f64 = 140.0;
const ACTIVE_SCHEDULE: [bool; 6] = [true, true, false, true, false, true];
const INACTIVE_SCHEDULE: [bool; 6] = [true, false, true, false, true, true];
const TIME_RESET_THRESHOLD: f64 = 3.0;

type WinOverride = Box<dyn Fn() -> Option<bool>>;

thread_local! {
    static SHIFT_TIMER: RefCell<Timer> = RefCell::new(Timer::new());
    static SHIFT_TIMER_OFFSET: Cell<f64> = const { Cell::new(0.0) };
    static ALLIANCE_WIN_OVERRIDE: RefCell<WinOverride> = RefCell::new(Box::new(|| None));
}

pub fn set_alliance_win_override(supplier: impl Fn() -> Option<bool> + 'static) {
    ALLIANCE_WIN_OVERRIDE.with(|s| *s.borrow_mut() = Box::new(supplier));
}

pub fn get_alliance_win_override() -> Option<bool> {
    ALLIANCE_WIN_OVERRIDE.with(|s| (s.borrow())())
}

fn other(alliance: Alliance) -> Alliance {
    if alliance == Alliance::Blue {
        Alliance::Red
    } else {
        Alliance::Blue
    }
}

pub fn get_first_active_alliance() -> Alliance {
    first_active_alliance_for(
        driver_station::alliance().unwrap_or(Alliance::Blue),
        get_alliance_win_override(),
        &driver_station::game_specific_message(),
    )
}

pub fn first_active_alliance_for(
    alliance: Alliance,
    win_override: Option<bool>,
    message: &str,
) -> Alliance {
    if let Some(won) = win_override {
        return if won { other(alliance) } else { alliance };
    }

    // The FMS game message names the alliance whose HUB is inactive in shift 1, so the
    // other one, the alliance active in shifts 2 and 4, is the first active alliance
    match message.chars().next() {
        Some('R') => return Alliance::Blue,
        Some('B') => return Alliance::Red,
        _ => {}
    }

    // Nothing from the FMS, so assume the opponent is active first
    other(alliance)
}

/// Starts the timer at the beginning of teleop.
pub fn initialize() {
    SHIFT_TIMER_OFFSET.with(|o| o.set(0.0));
    SHIFT_TIMER.with(|t| t.borrow_mut().restart());
}

fn get_schedule() -> &'static [bool; 6] {
    let alliance = driver_station::alliance().unwrap_or(Alliance::Blue);
    schedule_for(get_first_active_alliance(), alliance)
}

pub fn schedule_for(first_active: Alliance, alliance: Alliance) -> &'static [bool; 6] {
    if first_active == alliance {
        &ACTIVE_SCHEDULE
    } else {
        &INACTIVE_SCHEDULE
    }
}

/// What `get_shift_info` reads from the timer and the driver station.
#[derive(Debug, Clone, Copy)]
pub struct ClockInputs {
    pub timer_value: f64,
    pub match_time: f64,
    pub autonomous_enabled: bool,
    pub enabled: bool,
    pub fms_attached: bool,
}

fn read_clock() -> ClockInputs {
    ClockInputs {
        timer_value: SHIFT_TIMER.with(|t| t.borrow().get()),
        match_time: driver_station::match_time(),
        autonomous_enabled: driver_station::is_autonomous_enabled(),
        enabled: driver_station::is_enabled(),
        fms_attached: driver_station::is_fms_attached(),
    }
}

fn get_shift_info(
    current_schedule: &[bool; 6],
    shift_start_times: &[f64; 6],
    shift_end_times: &[f64; 6],
) -> ShiftInfo {
    let inputs = read_clock();
    let mut offset = SHIFT_TIMER_OFFSET.with(Cell::get);
    let info = compute_shift_info(
        &mut offset,
        inputs,
        current_schedule,
        shift_start_times,
        shift_end_times,
    );
    SHIFT_TIMER_OFFSET.with(|o| o.set(offset));
    info
}

/// `offset` is the timer offset, resynced to the FMS clock when the two drift apart.
pub fn compute_shift_info(
    offset: &mut f64,
    inputs: ClockInputs,
    current_schedule: &[bool; 6],
    shift_start_times: &[f64; 6],
    shift_end_times: &[f64; 6],
) -> ShiftInfo {
    let timer_value = inputs.timer_value;
    let mut current_time = timer_value - *offset;
    let mut state_time_elapsed = current_time;
    let mut state_time_remaining = 0.0;
    let mut active = false;
    let mut current_shift = ShiftEnum::Disabled;
    let field_teleop_time = 140.0 - inputs.match_time;

    if inputs.autonomous_enabled {
        state_time_elapsed = current_time;
        state_time_remaining = AUTO_END_TIME - current_time;
        active = true;
        current_shift = ShiftEnum::Auto;
    } else if inputs.enabled {
        // Resync the offset when the local timer drifts from the FMS match clock
        if (field_teleop_time - current_time).abs() >= TIME_RESET_THRESHOLD
            && field_teleop_time <= 135.0
            && inputs.fms_attached
        {
            *offset += current_time - field_teleop_time;
            current_time = timer_value - *offset;
        }
        let mut current_shift_index: Option<usize> = None;
        for i in 0..shift_start_times.len() {
            if current_time >= shift_start_times[i] && current_time < shift_end_times[i] {
                current_shift_index = Some(i);
                break;
            }
        }
        // Past the last shift, treat as endgame
        let index = current_shift_index.unwrap_or(shift_start_times.len() - 1);

        state_time_elapsed = current_time - shift_start_times[index];
        state_time_remaining = shift_end_times[index] - current_time;

        // Two adjacent shifts with the same active state read as one continuous phase
        if index > 0 && current_schedule[index] == current_schedule[index - 1] {
            state_time_elapsed = current_time - shift_start_times[index - 1];
        }

        if index < shift_end_times.len() - 1
            && current_schedule[index] == current_schedule[index + 1]
        {
            state_time_remaining = shift_end_times[index + 1] - current_time;
        }

        active = current_schedule[index];
        current_shift = SHIFTS_ENUMS[index];
    }
    ShiftInfo {
        current_shift,
        elapsed_time: state_time_elapsed,
        remaining_time: state_time_remaining,
        active,
    }
}

pub fn get_official_shift_info() -> ShiftInfo {
    get_shift_info(get_schedule(), &SHIFT_START_TIMES, &SHIFT_END_TIMES)
}

/// Shift boundaries moved by the active fudge times, for the schedule of this alliance.
pub fn shifted_times(shift_schedule: &[bool; 6]) -> ([f64; 6], [f64; 6]) {
    if shift_schedule[1] {
        (
            [
                0.0,
                10.0,
                35.0 + ENDING_ACTIVE_FUDGE,
                60.0 + APPROACHING_ACTIVE_FUDGE,
                85.0 + ENDING_ACTIVE_FUDGE,
                110.0 + APPROACHING_ACTIVE_FUDGE,
            ],
            [
                10.0,
                35.0 + ENDING_ACTIVE_FUDGE,
                60.0 + APPROACHING_ACTIVE_FUDGE,
                85.0 + ENDING_ACTIVE_FUDGE,
                110.0 + APPROACHING_ACTIVE_FUDGE,
                140.0,
            ],
        )
    } else {
        (
            [
                0.0,
                10.0 + ENDING_ACTIVE_FUDGE,
                35.0 + APPROACHING_ACTIVE_FUDGE,
                60.0 + ENDING_ACTIVE_FUDGE,
                85.0 + APPROACHING_ACTIVE_FUDGE,
                110.0,
            ],
            [
                10.0 + ENDING_ACTIVE_FUDGE,
                35.0 + APPROACHING_ACTIVE_FUDGE,
                60.0 + ENDING_ACTIVE_FUDGE,
                85.0 + APPROACHING_ACTIVE_FUDGE,
                110.0,
                140.0,
            ],
        )
    }
}

pub fn get_shifted_shift_info() -> ShiftInfo {
    let shift_schedule = get_schedule();
    let (starts, ends) = shifted_times(shift_schedule);
    get_shift_info(shift_schedule, &starts, &ends)
}

#[cfg(test)]
mod tests {
    use super::*;
    use golden::check;

    fn alliance(flag: f64) -> Alliance {
        if flag == 0.0 {
            Alliance::Blue
        } else {
            Alliance::Red
        }
    }

    fn shift_ordinal(shift: ShiftEnum) -> f64 {
        SHIFTS_ENUMS.iter().position(|s| *s == shift).unwrap_or(0) as f64
    }

    #[test]
    fn matches_java_shift_helpers() {
        check("shift_helpers", 1e-9, |op, i| {
            Some(match op {
                // alliance, override (-1 none), message first char code (0 none)
                "first_active_alliance" => {
                    let over = match i[1] as i32 {
                        -1 => None,
                        v => Some(v == 1),
                    };
                    let message = match i[2] as u32 {
                        0 => String::new(),
                        c => char::from_u32(c).map(String::from).unwrap_or_default(),
                    };
                    let first = first_active_alliance_for(alliance(i[0]), over, &message);
                    vec![if first == Alliance::Blue { 0.0 } else { 1.0 }]
                }
                // alliance, first active, which (0 official, 1 shifted), enabled, autonomous,
                // fms, match time, timer, offset
                "shift_info" => {
                    let schedule = schedule_for(alliance(i[1]), alliance(i[0]));
                    let (starts, ends) = if i[2] == 0.0 {
                        (SHIFT_START_TIMES, SHIFT_END_TIMES)
                    } else {
                        shifted_times(schedule)
                    };
                    let mut offset = i[8];
                    let info = compute_shift_info(
                        &mut offset,
                        ClockInputs {
                            timer_value: i[7],
                            match_time: i[6],
                            autonomous_enabled: i[4] != 0.0 && i[3] != 0.0,
                            enabled: i[3] != 0.0,
                            fms_attached: i[5] != 0.0,
                        },
                        schedule,
                        &starts,
                        &ends,
                    );
                    vec![
                        shift_ordinal(info.current_shift),
                        info.elapsed_time,
                        info.remaining_time,
                        f64::from(u8::from(info.active)),
                        offset,
                    ]
                }
                _ => return None,
            })
        });
    }

    fn clock(timer_value: f64) -> ClockInputs {
        ClockInputs {
            timer_value,
            match_time: -1.0,
            autonomous_enabled: false,
            enabled: true,
            fms_attached: false,
        }
    }

    #[test]
    fn disabled_robot_reports_the_disabled_shift() {
        let mut offset = 0.0;
        let info = compute_shift_info(
            &mut offset,
            ClockInputs {
                enabled: false,
                ..clock(50.0)
            },
            &ACTIVE_SCHEDULE,
            &SHIFT_START_TIMES,
            &SHIFT_END_TIMES,
        );
        assert_eq!(info.current_shift, ShiftEnum::Disabled);
        assert!(!info.active);
    }

    #[test]
    fn adjacent_shifts_with_one_state_read_as_one_phase() {
        let mut offset = 0.0;
        // Shift 1 and the transition are both active on the active schedule.
        let info = compute_shift_info(
            &mut offset,
            clock(12.0),
            &ACTIVE_SCHEDULE,
            &SHIFT_START_TIMES,
            &SHIFT_END_TIMES,
        );
        assert_eq!(info.current_shift, ShiftEnum::Shift1);
        assert_eq!(info.elapsed_time, 12.0);
        assert_eq!(info.remaining_time, 23.0);
    }

    #[test]
    fn past_the_last_shift_counts_as_endgame() {
        let mut offset = 0.0;
        let info = compute_shift_info(
            &mut offset,
            clock(150.0),
            &ACTIVE_SCHEDULE,
            &SHIFT_START_TIMES,
            &SHIFT_END_TIMES,
        );
        assert_eq!(info.current_shift, ShiftEnum::Endgame);
    }

    #[test]
    fn fms_clock_drift_moves_the_offset() {
        let mut offset = 0.0;
        compute_shift_info(
            &mut offset,
            ClockInputs {
                match_time: 100.0,
                fms_attached: true,
                ..clock(50.0)
            },
            &ACTIVE_SCHEDULE,
            &SHIFT_START_TIMES,
            &SHIFT_END_TIMES,
        );
        assert_eq!(offset, 10.0);
    }

    #[test]
    fn first_active_alliance_follows_the_override_then_the_message() {
        assert_eq!(
            first_active_alliance_for(Alliance::Blue, Some(true), "R"),
            Alliance::Red
        );
        assert_eq!(
            first_active_alliance_for(Alliance::Blue, Some(false), "R"),
            Alliance::Blue
        );
        assert_eq!(
            first_active_alliance_for(Alliance::Blue, None, "R"),
            Alliance::Blue
        );
        assert_eq!(
            first_active_alliance_for(Alliance::Red, None, "B"),
            Alliance::Red
        );
        assert_eq!(
            first_active_alliance_for(Alliance::Blue, None, ""),
            Alliance::Red
        );
    }

    #[test]
    fn schedule_matches_the_first_active_alliance() {
        assert_eq!(
            schedule_for(Alliance::Blue, Alliance::Blue),
            &ACTIVE_SCHEDULE
        );
        assert_eq!(
            schedule_for(Alliance::Red, Alliance::Blue),
            &INACTIVE_SCHEDULE
        );
    }
}
