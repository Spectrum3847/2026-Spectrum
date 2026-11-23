//! Java `DriverStation`: cached control word, match info and joystick data.
//!
//! [`refresh_data`] reads the HAL once per robot loop. Everything else reads the cache, so a
//! robot sees one consistent snapshot per loop. Button pressed and released flags latch on
//! refresh and clear when read, as in Java.

use crate::hal::{self, or_report};
use crate::robot_controller;
use std::panic::Location;
use std::sync::{Mutex, MutexGuard, PoisonError};

pub const JOYSTICK_PORTS: usize = 6;
pub const MAX_AXES: usize = hal::MAX_JOYSTICK_AXES;
pub const MAX_POVS: usize = hal::MAX_JOYSTICK_POVS;

const UNPLUGGED_MESSAGE_INTERVAL: f64 = 1.0;

/// `HAL_ControlWord` flags.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ControlWord(u32);

impl ControlWord {
    const ENABLED: u32 = 1 << 0;
    const AUTONOMOUS: u32 = 1 << 1;
    const TEST: u32 = 1 << 2;
    const ESTOP: u32 = 1 << 3;
    const FMS_ATTACHED: u32 = 1 << 4;
    const DS_ATTACHED: u32 = 1 << 5;

    pub fn from_bits(bits: u32) -> Self {
        Self(bits)
    }

    pub fn new(enabled: bool, autonomous: bool, test: bool, ds_attached: bool) -> Self {
        let mut bits = 0;
        for (on, bit) in [
            (enabled, Self::ENABLED),
            (autonomous, Self::AUTONOMOUS),
            (test, Self::TEST),
            (ds_attached, Self::DS_ATTACHED),
        ] {
            if on {
                bits |= bit;
            }
        }
        Self(bits)
    }

    pub fn with_fms_attached(self) -> Self {
        Self(self.0 | Self::FMS_ATTACHED)
    }

    pub fn with_estop(self) -> Self {
        Self(self.0 | Self::ESTOP)
    }

    pub fn enabled(self) -> bool {
        self.0 & Self::ENABLED != 0
    }

    pub fn autonomous(self) -> bool {
        self.0 & Self::AUTONOMOUS != 0
    }

    pub fn test(self) -> bool {
        self.0 & Self::TEST != 0
    }

    pub fn estop(self) -> bool {
        self.0 & Self::ESTOP != 0
    }

    pub fn fms_attached(self) -> bool {
        self.0 & Self::FMS_ATTACHED != 0
    }

    pub fn ds_attached(self) -> bool {
        self.0 & Self::DS_ATTACHED != 0
    }

    /// Java `DSControlWord.isEnabled`: enabled and a driver station is attached.
    pub fn is_enabled(self) -> bool {
        self.enabled() && self.ds_attached()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Alliance {
    Red,
    Blue,
}

/// `HAL_AllianceStationID`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AllianceStation {
    Unknown,
    Red1,
    Red2,
    Red3,
    Blue1,
    Blue2,
    Blue3,
}

impl AllianceStation {
    pub fn from_hal(value: i32) -> Self {
        match value {
            1 => Self::Red1,
            2 => Self::Red2,
            3 => Self::Red3,
            4 => Self::Blue1,
            5 => Self::Blue2,
            6 => Self::Blue3,
            _ => Self::Unknown,
        }
    }

    pub fn alliance(self) -> Option<Alliance> {
        match self {
            Self::Unknown => None,
            Self::Red1 | Self::Red2 | Self::Red3 => Some(Alliance::Red),
            Self::Blue1 | Self::Blue2 | Self::Blue3 => Some(Alliance::Blue),
        }
    }

    /// Driver station position 1 to 3.
    pub fn location(self) -> Option<u8> {
        match self {
            Self::Unknown => None,
            Self::Red1 | Self::Blue1 => Some(1),
            Self::Red2 | Self::Blue2 => Some(2),
            Self::Red3 | Self::Blue3 => Some(3),
        }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum MatchType {
    #[default]
    None,
    Practice,
    Qualification,
    Elimination,
}

impl MatchType {
    pub fn from_hal(value: i32) -> Self {
        match value {
            1 => Self::Practice,
            2 => Self::Qualification,
            3 => Self::Elimination,
            _ => Self::None,
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct MatchInfo {
    pub event_name: String,
    pub match_type: MatchType,
    pub match_number: u16,
    pub replay_number: u8,
    pub game_specific_message: String,
}

impl MatchInfo {
    fn from_raw(raw: &hal::MatchInfoRaw) -> Self {
        let name_len = raw
            .event_name
            .iter()
            .position(|&c| c == 0)
            .unwrap_or(raw.event_name.len());
        let name = &raw.event_name[..name_len];
        let size = usize::from(raw.game_specific_message_size).min(raw.game_specific_message.len());
        Self {
            event_name: String::from_utf8_lossy(name).into_owned(),
            match_type: MatchType::from_hal(raw.match_type),
            match_number: raw.match_number,
            replay_number: raw.replay_number,
            game_specific_message: String::from_utf8_lossy(&raw.game_specific_message[..size])
                .into_owned(),
        }
    }
}

/// Axes, POVs and buttons of one joystick port.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JoystickData {
    pub axes: [f32; MAX_AXES],
    pub axis_count: usize,
    pub povs: [i16; MAX_POVS],
    pub pov_count: usize,
    pub buttons: u32,
    pub button_count: u8,
}

impl JoystickData {
    pub const EMPTY: Self = Self {
        axes: [0.0; MAX_AXES],
        axis_count: 0,
        povs: [-1; MAX_POVS],
        pov_count: 0,
        buttons: 0,
        button_count: 0,
    };
}

/// One refresh worth of HAL data.
#[derive(Debug, Clone)]
pub struct Snapshot {
    pub sticks: [JoystickData; JOYSTICK_PORTS],
    pub match_info: MatchInfo,
    pub control_word: ControlWord,
}

struct State {
    sticks: [JoystickData; JOYSTICK_PORTS],
    pressed: [u32; JOYSTICK_PORTS],
    released: [u32; JOYSTICK_PORTS],
    control_word: ControlWord,
    match_info: MatchInfo,
    silence_warning: bool,
    next_message_time: f64,
}

impl State {
    const fn new() -> Self {
        Self {
            sticks: [JoystickData::EMPTY; JOYSTICK_PORTS],
            pressed: [0; JOYSTICK_PORTS],
            released: [0; JOYSTICK_PORTS],
            control_word: ControlWord(0),
            match_info: MatchInfo {
                event_name: String::new(),
                match_type: MatchType::None,
                match_number: 0,
                replay_number: 0,
                game_specific_message: String::new(),
            },
            silence_warning: false,
            next_message_time: 0.0,
        }
    }

    /// Latches button edges against the previous snapshot, then replaces the cache.
    fn apply(&mut self, snapshot: Snapshot) {
        let edges = self.pressed.iter_mut().zip(&mut self.released);
        for ((pressed, released), (old, new)) in edges.zip(self.sticks.iter().zip(&snapshot.sticks))
        {
            *pressed |= !old.buttons & new.buttons;
            *released |= old.buttons & !new.buttons;
        }
        self.sticks = snapshot.sticks;
        self.match_info = snapshot.match_info;
        self.control_word = snapshot.control_word;
    }

    // The getters below return None when the port lacks that input; the caller reports it
    // and uses a default.

    fn button(&self, stick: usize, button: u32) -> Option<bool> {
        let data = &self.sticks[stick];
        (button <= u32::from(data.button_count)).then(|| data.buttons & (1 << (button - 1)) != 0)
    }

    fn take_edge(&mut self, stick: usize, button: u32, pressed: bool) -> Option<bool> {
        if button > u32::from(self.sticks[stick].button_count) {
            return None;
        }
        let flags = if pressed {
            &mut self.pressed[stick]
        } else {
            &mut self.released[stick]
        };
        let mask = 1 << (button - 1);
        let was_set = *flags & mask != 0;
        *flags &= !mask;
        Some(was_set)
    }

    fn axis(&self, stick: usize, axis: usize) -> Option<f64> {
        let data = &self.sticks[stick];
        (axis < data.axis_count).then(|| f64::from(data.axes[axis]))
    }

    fn pov(&self, stick: usize, pov: usize) -> Option<i32> {
        let data = &self.sticks[stick];
        (pov < data.pov_count).then(|| i32::from(data.povs[pov]))
    }

    /// True when a joystick message may print now. Warnings obey the silence flag, errors do
    /// not. Both share one one-second throttle.
    fn take_message_slot(&mut self, now: f64, is_warning: bool) -> bool {
        if is_warning && !(self.control_word.fms_attached() || !self.silence_warning) {
            return false;
        }
        if now > self.next_message_time {
            self.next_message_time = now + UNPLUGGED_MESSAGE_INTERVAL;
            true
        } else {
            false
        }
    }
}

static STATE: Mutex<State> = Mutex::new(State::new());

fn state() -> MutexGuard<'static, State> {
    STATE.lock().unwrap_or_else(PoisonError::into_inner)
}

fn check_stick(stick: i32) -> usize {
    assert!(
        (0..JOYSTICK_PORTS as i32).contains(&stick),
        "Joystick index is out of range, should be 0-5"
    );
    stick as usize
}

fn report_unplugged(is_warning: bool, message: String) {
    let now = robot_controller::time_seconds();
    if state().take_message_slot(now, is_warning) {
        if is_warning {
            report_warning(&message);
        } else {
            report_error(&message);
        }
    }
}

const BUTTON_INDEX_ERROR: &str = "Button indexes begin at 1 in WPILib for C++ and Java\n";

// Reporting

#[track_caller]
pub fn report_error(message: &str) {
    send(true, message, false);
}

#[track_caller]
pub fn report_warning(message: &str) {
    send(false, message, false);
}

/// Like [`report_error`] with a backtrace in the call stack field.
#[track_caller]
pub fn report_error_with_trace(message: &str) {
    send(true, message, true);
}

#[track_caller]
pub fn report_warning_with_trace(message: &str) {
    send(false, message, true);
}

#[track_caller]
fn send(is_error: bool, message: &str, trace: bool) {
    let at = Location::caller();
    hal::send_error(
        is_error,
        1,
        message,
        &format!("{}:{}", at.file(), at.line()),
        &hal::trace_text(trace),
        true,
    );
}

// Refresh

fn read_snapshot() -> Snapshot {
    let mut sticks = [JoystickData::EMPTY; JOYSTICK_PORTS];
    for (i, data) in sticks.iter_mut().enumerate() {
        let stick = i as i32;
        let axes = hal::joystick_axes(stick);
        let povs = hal::joystick_povs(stick);
        let buttons = hal::joystick_buttons(stick);
        data.axes = axes.axes;
        data.axis_count = usize::try_from(axes.count).unwrap_or(0).min(MAX_AXES);
        data.povs = povs.povs;
        data.pov_count = usize::try_from(povs.count).unwrap_or(0).min(MAX_POVS);
        data.buttons = buttons.buttons;
        data.button_count = buttons.count;
    }
    Snapshot {
        sticks,
        match_info: MatchInfo::from_raw(&hal::match_info()),
        control_word: ControlWord::from_bits(hal::control_word().bits),
    }
}

/// Pulls the latest driver station data into the cache. The robot loop calls this first.
pub fn refresh_data() {
    hal::refresh_ds_data();
    let snapshot = read_snapshot();
    state().apply(snapshot);
}

// Control word

pub fn control_word() -> ControlWord {
    state().control_word
}

pub fn is_enabled() -> bool {
    control_word().is_enabled()
}

pub fn is_disabled() -> bool {
    !is_enabled()
}

pub fn is_estopped() -> bool {
    control_word().estop()
}

pub fn is_autonomous() -> bool {
    control_word().autonomous()
}

/// Autonomous and enabled. Unlike `ControlWord::is_enabled`, this ignores whether a driver
/// station is attached, as Java's `DriverStation.isAutonomousEnabled` does.
pub fn is_autonomous_enabled() -> bool {
    let word = control_word();
    word.autonomous() && word.enabled()
}

pub fn is_teleop() -> bool {
    let word = control_word();
    !(word.autonomous() || word.test())
}

pub fn is_teleop_enabled() -> bool {
    let word = control_word();
    !word.autonomous() && !word.test() && word.enabled()
}

pub fn is_test() -> bool {
    control_word().test()
}

pub fn is_test_enabled() -> bool {
    let word = control_word();
    word.test() && word.enabled()
}

pub fn is_ds_attached() -> bool {
    control_word().ds_attached()
}

pub fn is_fms_attached() -> bool {
    control_word().fms_attached()
}

// Match

pub fn match_info() -> MatchInfo {
    state().match_info.clone()
}

pub fn game_specific_message() -> String {
    state().match_info.game_specific_message.clone()
}

pub fn event_name() -> String {
    state().match_info.event_name.clone()
}

pub fn match_type() -> MatchType {
    state().match_info.match_type
}

pub fn match_number() -> u16 {
    state().match_info.match_number
}

pub fn replay_number() -> u8 {
    state().match_info.replay_number
}

/// The station from the HAL, not the cache, as Java reads it.
#[track_caller]
pub fn raw_alliance_station() -> AllianceStation {
    AllianceStation::from_hal(or_report(hal::alliance_station()))
}

pub fn alliance() -> Option<Alliance> {
    raw_alliance_station().alliance()
}

pub fn location() -> Option<u8> {
    raw_alliance_station().location()
}

/// Seconds left in the current period, or -1 when unknown.
#[track_caller]
pub fn match_time() -> f64 {
    or_report(hal::match_time())
}

// Joysticks

pub fn silence_joystick_connection_warning(silence: bool) {
    state().silence_warning = silence;
}

pub fn is_joystick_connection_warning_silenced() -> bool {
    let st = state();
    !st.control_word.fms_attached() && st.silence_warning
}

fn unplugged_warning(what: &str, index: impl std::fmt::Display, stick: usize) {
    report_unplugged(
        true,
        format!(
            "Joystick {what} {index} on port {stick} not available, check if controller is plugged in"
        ),
    );
}

/// Button state. Buttons count from 1.
///
/// # Panics
/// When `stick` is not 0 to 5.
pub fn stick_button(stick: i32, button: i32) -> bool {
    let stick = check_stick(stick);
    let Some(index) = button_index(button) else {
        return false;
    };
    let result = state().button(stick, index);
    result.unwrap_or_else(|| {
        unplugged_warning("Button", button, stick);
        false
    })
}

fn button_index(button: i32) -> Option<u32> {
    let index = u32::try_from(button).ok().filter(|&b| b > 0);
    if index.is_none() {
        report_unplugged(false, BUTTON_INDEX_ERROR.to_owned());
    }
    index
}

/// True once if the button went down since the last call. Clears on read.
///
/// # Panics
/// When `stick` is not 0 to 5.
pub fn stick_button_pressed(stick: i32, button: i32) -> bool {
    stick_button_edge(stick, button, true)
}

/// True once if the button went up since the last call. Clears on read.
///
/// # Panics
/// When `stick` is not 0 to 5.
pub fn stick_button_released(stick: i32, button: i32) -> bool {
    stick_button_edge(stick, button, false)
}

fn stick_button_edge(stick: i32, button: i32, pressed: bool) -> bool {
    let Some(index) = button_index(button) else {
        return false;
    };
    let stick = check_stick(stick);
    let result = state().take_edge(stick, index, pressed);
    result.unwrap_or_else(|| {
        unplugged_warning("Button", button, stick);
        false
    })
}

/// # Panics
/// When `stick` is not 0 to 5 or `axis` is not 0 to 11.
pub fn stick_axis(stick: i32, axis: i32) -> f64 {
    let stick = check_stick(stick);
    assert!(
        (0..MAX_AXES as i32).contains(&axis),
        "Joystick axis is out of range"
    );
    let result = state().axis(stick, axis as usize);
    result.unwrap_or_else(|| {
        unplugged_warning("axis", axis, stick);
        0.0
    })
}

/// POV angle in degrees, or -1 when centered or missing.
///
/// # Panics
/// When `stick` is not 0 to 5 or `pov` is not 0 to 11.
pub fn stick_pov(stick: i32, pov: i32) -> i32 {
    let stick = check_stick(stick);
    assert!(
        (0..MAX_POVS as i32).contains(&pov),
        "Joystick POV is out of range"
    );
    let result = state().pov(stick, pov as usize);
    result.unwrap_or_else(|| {
        unplugged_warning("POV", pov, stick);
        -1
    })
}

pub fn stick_buttons(stick: i32) -> u32 {
    state().sticks[check_stick(stick)].buttons
}

pub fn stick_axis_count(stick: i32) -> usize {
    state().sticks[check_stick(stick)].axis_count
}

pub fn stick_pov_count(stick: i32) -> usize {
    state().sticks[check_stick(stick)].pov_count
}

pub fn stick_button_count(stick: i32) -> usize {
    usize::from(state().sticks[check_stick(stick)].button_count)
}

pub fn is_joystick_connected(stick: i32) -> bool {
    stick_axis_count(stick) > 0 || stick_button_count(stick) > 0 || stick_pov_count(stick) > 0
}

pub fn joystick_is_xbox(stick: i32) -> bool {
    hal::joystick_is_xbox(check_stick(stick) as i32)
}

pub fn joystick_type(stick: i32) -> i32 {
    hal::joystick_type(check_stick(stick) as i32)
}

/// Sets HID outputs and rumble. Rumble values are 0 to 65535.
pub fn set_joystick_outputs(stick: i32, outputs: i64, left_rumble: i32, right_rumble: i32) {
    hal::set_joystick_outputs(
        check_stick(stick) as i32,
        outputs,
        left_rumble,
        right_rumble,
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stick_with_buttons(buttons: u32, count: u8) -> JoystickData {
        JoystickData {
            buttons,
            button_count: count,
            ..JoystickData::EMPTY
        }
    }

    fn snapshot(stick0: JoystickData) -> Snapshot {
        let mut sticks = [JoystickData::EMPTY; JOYSTICK_PORTS];
        sticks[0] = stick0;
        Snapshot {
            sticks,
            match_info: MatchInfo::default(),
            control_word: ControlWord::default(),
        }
    }

    #[test]
    fn control_word_bits() {
        let word = ControlWord::from_bits(0b10_0011);
        assert!(word.enabled() && word.autonomous() && word.ds_attached());
        assert!(!word.test() && !word.estop() && !word.fms_attached());
        assert!(word.is_enabled());
        assert!(!ControlWord::new(true, false, false, false).is_enabled());
        assert!(ControlWord::default().with_fms_attached().fms_attached());
        assert!(ControlWord::default().with_estop().estop());
    }

    #[test]
    fn alliance_station_mapping() {
        assert_eq!(AllianceStation::from_hal(0).alliance(), None);
        assert_eq!(AllianceStation::from_hal(2).alliance(), Some(Alliance::Red));
        assert_eq!(AllianceStation::from_hal(2).location(), Some(2));
        assert_eq!(
            AllianceStation::from_hal(6).alliance(),
            Some(Alliance::Blue)
        );
        assert_eq!(AllianceStation::from_hal(6).location(), Some(3));
        assert_eq!(AllianceStation::from_hal(99), AllianceStation::Unknown);
    }

    #[test]
    fn match_info_from_raw_uses_declared_sizes() {
        let mut raw = hal::MatchInfoRaw {
            event_name: [0; 64],
            match_type: 2,
            match_number: 14,
            replay_number: 1,
            game_specific_message: [0; 64],
            game_specific_message_size: 3,
        };
        for (slot, byte) in raw.event_name.iter_mut().zip(b"CMP\0junk") {
            *slot = *byte;
        }
        raw.game_specific_message[..5].copy_from_slice(b"RBGxx");
        let info = MatchInfo::from_raw(&raw);
        assert_eq!(info.event_name, "CMP");
        assert_eq!(info.match_type, MatchType::Qualification);
        assert_eq!((info.match_number, info.replay_number), (14, 1));
        assert_eq!(info.game_specific_message, "RBG");
    }

    #[test]
    fn pressed_latches_until_read() {
        let mut st = State::new();
        st.apply(snapshot(stick_with_buttons(0b000, 3)));
        st.apply(snapshot(stick_with_buttons(0b010, 3)));
        assert_eq!(st.button(0, 2), Some(true));
        assert_eq!(st.take_edge(0, 2, true), Some(true));
        assert_eq!(st.take_edge(0, 2, true), Some(false));
        assert_eq!(st.take_edge(0, 1, true), Some(false));
    }

    #[test]
    fn pressed_survives_a_quick_release_between_reads() {
        let mut st = State::new();
        st.apply(snapshot(stick_with_buttons(0, 3)));
        st.apply(snapshot(stick_with_buttons(0b1, 3)));
        st.apply(snapshot(stick_with_buttons(0, 3)));
        assert_eq!(st.button(0, 1), Some(false));
        assert_eq!(st.take_edge(0, 1, true), Some(true));
        assert_eq!(st.take_edge(0, 1, false), Some(true));
        assert_eq!(st.take_edge(0, 1, false), Some(false));
    }

    #[test]
    fn released_fires_on_falling_edge() {
        let mut st = State::new();
        st.apply(snapshot(stick_with_buttons(0b100, 3)));
        st.apply(snapshot(stick_with_buttons(0b000, 3)));
        assert_eq!(st.take_edge(0, 3, false), Some(true));
        assert_eq!(st.take_edge(0, 3, false), Some(false));
    }

    #[test]
    fn buttons_past_the_count_are_unavailable() {
        let mut st = State::new();
        st.apply(snapshot(stick_with_buttons(0b1, 1)));
        assert_eq!(st.button(0, 2), None);
        assert_eq!(st.take_edge(0, 2, true), None);
        assert_eq!(st.button(1, 1), None);
    }

    #[test]
    fn axes_and_povs_respect_counts() {
        let mut data = JoystickData::EMPTY;
        data.axes[1] = 0.5;
        data.axis_count = 2;
        data.povs[0] = 90;
        data.pov_count = 1;
        let mut st = State::new();
        st.apply(snapshot(data));
        assert_eq!(st.axis(0, 1), Some(0.5));
        assert_eq!(st.axis(0, 2), None);
        assert_eq!(st.pov(0, 0), Some(90));
        assert_eq!(st.pov(0, 1), None);
    }

    #[test]
    fn message_slot_throttles_to_one_per_second() {
        let mut st = State::new();
        assert!(st.take_message_slot(10.0, false));
        assert!(!st.take_message_slot(10.5, false));
        assert!(!st.take_message_slot(11.0, false));
        assert!(st.take_message_slot(11.1, false));
    }

    #[test]
    fn silenced_warnings_do_not_use_the_slot_but_errors_do() {
        let mut st = State::new();
        st.silence_warning = true;
        assert!(!st.take_message_slot(10.0, true));
        assert!(st.take_message_slot(10.0, false));
        st.control_word = ControlWord::default().with_fms_attached();
        assert!(st.take_message_slot(12.0, true));
    }
}
