//! The `FMSInfo` table Java's `DriverStation` publishes on every data refresh. Dashboards
//! read it for the match, alliance and control state.

use super::io::TopicIo;
use crate::driver_station::{self, AllianceStation};
use crate::hal;
use crate::nt::Value;

const TABLE: &str = "/FMSInfo";

/// What `FMSInfo` reports.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MatchData {
    pub is_red_alliance: bool,
    pub station_number: i64,
    pub event_name: String,
    pub game_specific_message: String,
    pub match_number: i64,
    pub replay_number: i64,
    pub match_type: i64,
    pub control_word: i64,
}

impl MatchData {
    /// What Java publishes before the first refresh.
    fn initial() -> Self {
        Self {
            is_red_alliance: true,
            station_number: 1,
            event_name: String::new(),
            game_specific_message: String::new(),
            match_number: 0,
            replay_number: 0,
            match_type: 0,
            control_word: 0,
        }
    }

    /// Reads the cached match info and the HAL alliance station and control word. An unknown
    /// station reads as red 3, as in Java.
    pub fn current() -> Self {
        let info = driver_station::match_info();
        let station = driver_station::raw_alliance_station();
        Self {
            is_red_alliance: !matches!(
                station,
                AllianceStation::Blue1 | AllianceStation::Blue2 | AllianceStation::Blue3
            ),
            station_number: match station {
                AllianceStation::Red1 | AllianceStation::Blue1 => 1,
                AllianceStation::Red2 | AllianceStation::Blue2 => 2,
                _ => 3,
            },
            event_name: info.event_name,
            game_specific_message: info.game_specific_message,
            match_number: i64::from(info.match_number),
            replay_number: i64::from(info.replay_number),
            match_type: info.match_type as i64,
            control_word: i64::from(hal::control_word().bits),
        }
    }
}

/// Publishes `FMSInfo`, writing only the values that changed.
pub struct MatchDataSender<I: TopicIo> {
    io: I,
    last: MatchData,
}

impl<I: TopicIo> MatchDataSender<I> {
    /// Announces every topic with its initial value.
    pub fn new(mut io: I) -> Self {
        let kind = Value::String("FMSInfo".to_owned());
        io.publish_with_properties(
            &format!("{TABLE}/.type"),
            "{\"SmartDashboard\":\"FMSInfo\"}",
            &kind,
        );
        io.set(&format!("{TABLE}/.type"), &kind);
        let last = MatchData::initial();
        let all = |name: &str, value: Value, io: &mut I| io.set(&format!("{TABLE}/{name}"), &value);
        all("GameSpecificMessage", Value::String(String::new()), &mut io);
        all("EventName", Value::String(String::new()), &mut io);
        all("MatchNumber", Value::Integer(0), &mut io);
        all("ReplayNumber", Value::Integer(0), &mut io);
        all("MatchType", Value::Integer(0), &mut io);
        all("IsRedAlliance", Value::Boolean(true), &mut io);
        all("StationNumber", Value::Integer(1), &mut io);
        all("FMSControlData", Value::Integer(0), &mut io);
        Self { io, last }
    }

    pub fn send(&mut self, now: &MatchData) {
        let last = &mut self.last;
        let io = &mut self.io;
        let mut put = |name: &str, value: Value| io.set(&format!("{TABLE}/{name}"), &value);
        if last.is_red_alliance != now.is_red_alliance {
            put("IsRedAlliance", Value::Boolean(now.is_red_alliance));
        }
        if last.station_number != now.station_number {
            put("StationNumber", Value::Integer(now.station_number));
        }
        if last.event_name != now.event_name {
            put("EventName", Value::String(now.event_name.clone()));
        }
        if last.game_specific_message != now.game_specific_message {
            put(
                "GameSpecificMessage",
                Value::String(now.game_specific_message.clone()),
            );
        }
        if last.match_number != now.match_number {
            put("MatchNumber", Value::Integer(now.match_number));
        }
        if last.replay_number != now.replay_number {
            put("ReplayNumber", Value::Integer(now.replay_number));
        }
        if last.match_type != now.match_type {
            put("MatchType", Value::Integer(now.match_type));
        }
        if last.control_word != now.control_word {
            put("FMSControlData", Value::Integer(now.control_word));
        }
        *last = now.clone();
    }

    /// Reads the current match data and sends the changes.
    pub fn refresh(&mut self) {
        let now = MatchData::current();
        self.send(&now);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dashboard::io::FakeIo;

    fn sender() -> (
        std::rc::Rc<std::cell::RefCell<FakeIo>>,
        MatchDataSender<std::rc::Rc<std::cell::RefCell<FakeIo>>>,
    ) {
        let fake = FakeIo::shared();
        (fake.clone(), MatchDataSender::new(fake))
    }

    #[test]
    fn initial_values_match_java() {
        let (fake, _) = sender();
        let io = fake.borrow();
        assert_eq!(io.text("/FMSInfo/.type"), "FMSInfo");
        assert_eq!(
            io.properties["/FMSInfo/.type"],
            "{\"SmartDashboard\":\"FMSInfo\"}"
        );
        assert_eq!(io.text("/FMSInfo/GameSpecificMessage"), "");
        assert_eq!(io.text("/FMSInfo/EventName"), "");
        assert_eq!(io.values["/FMSInfo/MatchNumber"], Value::Integer(0));
        assert_eq!(io.values["/FMSInfo/ReplayNumber"], Value::Integer(0));
        assert_eq!(io.values["/FMSInfo/MatchType"], Value::Integer(0));
        assert_eq!(io.values["/FMSInfo/IsRedAlliance"], Value::Boolean(true));
        assert_eq!(io.values["/FMSInfo/StationNumber"], Value::Integer(1));
        assert_eq!(io.values["/FMSInfo/FMSControlData"], Value::Integer(0));
    }

    #[test]
    fn only_changed_values_are_written() {
        let (fake, mut sender) = sender();
        fake.borrow_mut().writes.clear();
        let mut now = MatchData::initial();
        sender.send(&now);
        assert!(fake.borrow().writes.is_empty());

        now.is_red_alliance = false;
        now.station_number = 2;
        now.control_word = 33;
        now.event_name = "Event".into();
        sender.send(&now);
        let io = fake.borrow();
        assert_eq!(
            io.writes,
            [
                "/FMSInfo/IsRedAlliance",
                "/FMSInfo/StationNumber",
                "/FMSInfo/EventName",
                "/FMSInfo/FMSControlData"
            ]
        );
        assert_eq!(io.values["/FMSInfo/FMSControlData"], Value::Integer(33));
        assert_eq!(io.values["/FMSInfo/IsRedAlliance"], Value::Boolean(false));
    }

    #[test]
    fn a_value_is_written_once_per_change() {
        let (fake, mut sender) = sender();
        let mut now = MatchData::initial();
        now.match_number = 12;
        sender.send(&now);
        let after_first = fake.borrow().writes.len();
        sender.send(&now);
        assert_eq!(fake.borrow().writes.len(), after_first);
        now.match_number = 13;
        sender.send(&now);
        assert_eq!(
            fake.borrow().values["/FMSInfo/MatchNumber"],
            Value::Integer(13)
        );
    }
}
