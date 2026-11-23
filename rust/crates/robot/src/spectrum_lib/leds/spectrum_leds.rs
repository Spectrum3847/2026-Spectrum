//! CANdle-based addressable LED subsystem, with pattern factories for solids, stripes,
//! blinks, breathes, rainbows, chases, bounces, gradients, ombres, waves, and countdowns.
//!
//! Patterns come in two flavors. Hardware animations ([`SpectrumLEDs::blink`],
//! [`SpectrumLEDs::breathe`], [`SpectrumLEDs::rainbow`], [`SpectrumLEDs::scrolling_rainbow`],
//! [`SpectrumLEDs::chase`], [`SpectrumLEDs::bounce`], [`SpectrumLEDs::fire`],
//! [`SpectrumLEDs::rgb_cycle`]) run in the CANdle's firmware and cost nothing per loop.
//! Software patterns ([`SpectrumLEDs::solid`], [`SpectrumLEDs::stripe`],
//! [`SpectrumLEDs::gradient`], [`SpectrumLEDs::ombre`], [`SpectrumLEDs::wave`],
//! [`SpectrumLEDs::countdown`], [`SpectrumLEDs::switch_countdown`], [`SpectrumLEDs::edges`])
//! send a one-shot `SolidColor` that is resent every loop, and switching to one of them
//! clears the running animations.
//!
//! Several instances can share one physical `CANdle` by passing the same device to their
//! [`Config`] and taking non-overlapping `start_idx` and `num_leds` ranges, each with its
//! own animation slot.
//!
//! [`SpectrumLEDs::set_pattern`] returns a `Command` that applies the pattern for as long as
//! it runs and holds the priority it was given ([`SpectrumLEDs::check_priority`]).
//!
//! Java's `Leds extends SpectrumLEDs`. A Rust subsystem holds the `Rc<SpectrumLEDs>` that
//! [`SpectrumLEDs::new`] returns, which registers itself with the scheduler.

use crate::spectrum_lib::hardware::can_config_budget;
use commands::{Command, CommandScheduler, Commands, Subsystem, SubsystemId, Trigger};
use phoenix6::configs::{CANdleConfiguration, LEDConfigs};
use phoenix6::controls::{
    CandleControl, ColorFlowAnimation, EmptyAnimation, FireAnimation, LarsonAnimation,
    RainbowAnimation, RgbFadeAnimation, SingleFadeAnimation, SolidColor, StrobeAnimation,
};
use phoenix6::signals::{LarsonBounceValue, LossOfSignalBehaviorValue, RgbwColor, StripTypeValue};
use phoenix6::{CANdle, CanBus};
use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::time::{SystemTime, UNIX_EPOCH};
use wpilib::color::Color;
use wpilib::timer;

type ApplyFn = dyn Fn(&CANdle, i32, i32);

/// A LED pattern that drives a segment of a `CANdle` strip.
///
/// Applying receives the live `CANdle` device, the first LED index in this instance's
/// segment (`start_idx`), and the number of LEDs in the segment (`num_leds`). Hardware
/// animation patterns send an animation request, software patterns send `SolidColor`
/// requests (one-shot, resent each loop).
#[derive(Clone)]
pub struct CANdlePattern {
    apply: Rc<ApplyFn>,
    /// Marks a hardware animation, so [`SpectrumLEDs::set_pattern`] can see that the
    /// pattern has to clear the animation slots when it is swapped out.
    hardware_animation: bool,
}

impl CANdlePattern {
    /// A software pattern.
    pub fn new(apply: impl Fn(&CANdle, i32, i32) + 'static) -> Self {
        Self {
            apply: Rc::new(apply),
            hardware_animation: false,
        }
    }

    fn hardware_animation(apply: impl Fn(&CANdle, i32, i32) + 'static) -> Self {
        Self {
            apply: Rc::new(apply),
            hardware_animation: true,
        }
    }

    /// Apply this pattern to the given LED segment. `start_idx` is the first LED index
    /// (inclusive), where 0 includes the 8 onboard LEDs.
    pub fn apply_to(&self, candle: &CANdle, start_idx: i32, num_leds: i32) {
        (self.apply)(candle, start_idx, num_leds);
    }

    pub fn is_hardware_animation(&self) -> bool {
        self.hardware_animation
    }
}

/// A pattern that sends one request, built on the first call from the runtime segment and
/// resent every call after that.
fn lazy(
    request: impl Fn(i32, i32) -> Box<dyn CandleControl> + 'static,
) -> impl Fn(&CANdle, i32, i32) + 'static {
    let holder: RefCell<Option<Box<dyn CandleControl>>> = RefCell::new(None);
    move |candle, start_idx, num_leds| {
        let mut holder = holder.borrow_mut();
        let request = holder.get_or_insert_with(|| request(start_idx, num_leds));
        candle.set_control(&**request);
    }
}

/// [`lazy`] marked as a hardware animation.
fn lazy_anim(request: impl Fn(i32, i32) -> Box<dyn CandleControl> + 'static) -> CANdlePattern {
    CANdlePattern::hardware_animation(lazy(request))
}

struct PerLed {
    requests: Vec<SolidColor>,
    colors: Vec<RgbwColor>,
}

/// A software pattern that writes every LED individually each loop. `frame` fills in each
/// LED's color for one loop, from the segment length. Java built a fresh closure per loop
/// for this; the buffer is the same values without the allocation.
fn per_led(frame: impl Fn(usize, &mut [RgbwColor]) + 'static) -> CANdlePattern {
    let holder: RefCell<Option<PerLed>> = RefCell::new(None);
    CANdlePattern::new(move |candle, start_idx, num_leds| {
        let mut holder = holder.borrow_mut();
        let state = holder.get_or_insert_with(|| {
            let count = num_leds.max(0) as usize;
            PerLed {
                requests: (0..count as i32)
                    .map(|i| SolidColor::new(start_idx + i, start_idx + i))
                    .collect(),
                colors: vec![OFF; count],
            }
        });
        frame(state.colors.len(), &mut state.colors);
        for (request, color) in state.requests.iter_mut().zip(&state.colors) {
            request.color = *color;
            candle.set_control(request);
        }
    })
}

/// Configuration for a [`SpectrumLEDs`] subsystem instance.
///
/// Use [`Config::new`] for an instance that owns and configures its own `CANdle`, or
/// [`Config::with_shared_candle`] to share a device that is already configured. Lombok
/// getters and setters on the Java class are public fields.
#[derive(Debug, Clone)]
pub struct Config {
    /// Human-readable name used in telemetry.
    pub name: String,

    /// Whether this LED strip is physically connected to the robot.
    pub attached: bool,

    /// Pre-built `CANdle` to reuse. When set, `device_id` and `can_bus` are ignored and this
    /// instance applies no hardware configuration.
    pub shared_candle: Option<Rc<CANdle>>,

    /// CAN device ID used when no `shared_candle` is provided.
    pub device_id: i32,

    /// CAN bus used when no `shared_candle` is provided.
    pub can_bus: Option<CanBus>,

    /// First LED index (inclusive) in the strip. Use 0 to include the 8 onboard status LEDs
    /// on the CANdle board itself; use 8 to skip them.
    pub start_idx: i32,

    /// Number of LEDs in the segment owned by this instance.
    pub num_leds: i32,

    /// CANdle hardware animation slot (0-7) used by this instance's animation patterns. Each
    /// [`SpectrumLEDs`] instance sharing a single `CANdle` must use a distinct slot,
    /// otherwise their animations overwrite each other.
    pub animation_slot: i32,

    /// LED strip type (RGB, RGBW, GRB, etc.). Ignored when `shared_candle` is set.
    pub strip_type: StripTypeValue,

    /// Overall brightness scalar applied in hardware (0.0-1.0). Ignored when
    /// `shared_candle` is set.
    pub brightness: f64,

    /// Behavior of the strip when CAN signal is lost. Ignored when `shared_candle` is set.
    pub loss_of_signal_behavior: LossOfSignalBehaviorValue,
}

impl Config {
    /// Configures an instance that creates and owns its own `CANdle`, applying strip type,
    /// brightness, and loss-of-signal behavior to the hardware. `start_idx` starts at 8 so
    /// the onboard LEDs are skipped.
    ///
    /// `num_leds` is the number of LEDs on the external strip, not counting the 8 onboard
    /// LEDs.
    pub fn new(name: impl Into<String>, device_id: i32, num_leds: i32, can_bus: CanBus) -> Self {
        Self {
            device_id,
            can_bus: Some(can_bus),
            num_leds,
            start_idx: 8,
            ..Self::defaults(name.into())
        }
    }

    /// Configures a zone that shares an already-configured `CANdle`. No hardware
    /// configuration is applied.
    pub fn with_shared_candle(
        name: impl Into<String>,
        shared_candle: Rc<CANdle>,
        start_idx: i32,
        num_leds: i32,
    ) -> Self {
        Self {
            shared_candle: Some(shared_candle),
            start_idx,
            num_leds,
            ..Self::defaults(name.into())
        }
    }

    /// Java's field initializers.
    fn defaults(name: String) -> Self {
        Self {
            name,
            attached: true,
            shared_candle: None,
            device_id: 1,
            can_bus: None,
            start_idx: 0,
            num_leds: 0,
            animation_slot: 0,
            strip_type: StripTypeValue::RGB,
            brightness: 1.0,
            loss_of_signal_behavior: LossOfSignalBehaviorValue::DisableLEDs,
        }
    }
}

/// What the pattern commands share with the subsystem.
struct State {
    config: Config,
    candle: Rc<CANdle>,
    last_was_animation: Cell<bool>,
    command_priority: Cell<i32>,
}

pub struct SpectrumLEDs {
    id: SubsystemId,
    state: Rc<State>,

    /// Pattern shown when no other command wants this subsystem (an orange blink).
    pub default_pattern: CANdlePattern,

    /// The default command, which displays `default_pattern` at the lowest priority. A
    /// subsystem can install its own default command to replace it; query the installed one
    /// with `Subsystem::get_default_command`.
    pub default_command: Command,

    /// Active while the installed default command is the one running, which means no
    /// higher-priority pattern owns the subsystem.
    pub default_trigger: Trigger,

    /// Spectrum purple color constant (`RGB 130, 103, 185`).
    pub purple: Color,

    /// Convenience alias for `Color::k_white`.
    pub white: Color,
}

impl SpectrumLEDs {
    /// Configures the hardware (or reuses a shared device) and registers with the
    /// scheduler. Panics for a bad CAN device ID or a missing CAN bus, where Java throws.
    ///
    /// `config` holds the device, segment range, and strip type for this instance.
    pub fn new(config: Config) -> Rc<SpectrumLEDs> {
        let candle = match &config.shared_candle {
            Some(shared) => shared.clone(),
            None => {
                let can_bus = config
                    .can_bus
                    .clone()
                    .expect("a SpectrumLEDs config without a shared CANdle needs a CAN bus");
                let candle = CANdle::new(config.device_id, can_bus).unwrap_or_else(|code| {
                    panic!("Could not create CANdle {}: {code}", config.device_id)
                });
                let candle_config = CANdleConfiguration::default().with_led(
                    LEDConfigs::default()
                        .with_strip_type(config.strip_type)
                        .with_brightness_scalar(config.brightness)
                        .with_loss_of_signal_behavior(config.loss_of_signal_behavior),
                );
                can_config_budget::run(&format!("CANdle {}", config.device_id), |timeout| {
                    candle
                        .configurator()
                        .apply_with_timeout(&candle_config, timeout)
                });
                Rc::new(candle)
            }
        };

        let id = SubsystemId::new();
        let animation_slot = config.animation_slot;
        let state = Rc::new(State {
            config,
            candle,
            last_was_animation: Cell::new(false),
            command_priority: Cell::new(-1),
        });

        let default_pattern = blink_pattern(animation_slot, Color::k_orange(), 1.0);
        let default_command =
            set_pattern_command(id, &state, &default_pattern, -1).with_name("LEDs.defaultCommand");
        let default_trigger = Trigger::new(move || {
            let scheduler = CommandScheduler::get_instance();
            let current = scheduler.requiring(id);
            current.is_some() && current == scheduler.get_default_command(id)
        });

        let leds = Rc::new(SpectrumLEDs {
            id,
            state,
            default_pattern,
            default_command,
            default_trigger,
            purple: Color::from_rgb(130, 103, 185),
            white: Color::k_white(),
        });
        leds.set_default_command(leds.default_command.clone());
        CommandScheduler::get_instance().register_subsystem(leds.clone());
        leds
    }

    pub fn get_config(&self) -> &Config {
        &self.state.config
    }

    /// The `CANdle` this instance drives, either owned or shared.
    pub fn get_candle(&self) -> &Rc<CANdle> {
        &self.state.candle
    }

    pub fn is_attached(&self) -> bool {
        self.state.config.attached
    }

    /// True when the most recently applied pattern was a hardware animation running in
    /// firmware, false when it was a software `SolidColor` pattern.
    pub fn is_animating(&self) -> bool {
        self.state.last_was_animation.get()
    }

    /// Priority of the pattern command currently running; [`Self::set_pattern`] stores this
    /// while a command runs and resets it to -1 when the command ends.
    pub fn get_command_priority(&self) -> i32 {
        self.state.command_priority.get()
    }

    pub fn set_command_priority(&self, priority: i32) {
        self.state.command_priority.set(priority);
    }

    /// Name of the command currently applying a pattern, or "None" when none is running.
    pub fn get_current_command_name(&self) -> String {
        match self.get_current_command() {
            Some(command) => command.get_name(),
            None => "None".to_owned(),
        }
    }

    /// Returns a trigger that is true while the running pattern command's priority is at or
    /// below `priority`, for gating a lower-priority pattern out of the way of a
    /// higher-priority one.
    pub fn check_priority(&self, priority: i32) -> Trigger {
        let state = self.state.clone();
        Trigger::new(move || state.command_priority.get() <= priority)
    }

    /// Returns a command that applies `pattern` to the segment for as long as it runs and
    /// holds `priority` while it does. The command runs while the robot is disabled.
    ///
    /// Switching from a hardware animation to a software `SolidColor` pattern clears the
    /// active animation slots before the first software write.
    pub fn set_pattern(&self, pattern: &CANdlePattern, priority: i32) -> Command {
        set_pattern_command(self.id, &self.state, pattern, priority)
    }

    /// [`Self::set_pattern`] at priority 0.
    pub fn set_pattern_default_priority(&self, pattern: &CANdlePattern) -> Command {
        self.set_pattern(pattern, 0)
    }

    /// Blinks between `color` and off. Each half-cycle, on and off, lasts `on_time_secs`
    /// seconds, so the frame rate is one over that.
    pub fn blink(&self, color: Color, on_time_secs: f64) -> CANdlePattern {
        blink_pattern(self.state.config.animation_slot, color, on_time_secs)
    }

    /// Fades between `color` and off and back, over `period_secs`. Each animation frame moves
    /// brightness by 1%, so a full cycle needs 200 frames and the frame rate is 200 over the
    /// period.
    pub fn breathe(&self, color: Color, period_secs: f64) -> CANdlePattern {
        let rgbw = to_rgbw(color);
        let slot = self.state.config.animation_slot;
        lazy_anim(move |start_idx, num_leds| {
            Box::new(
                SingleFadeAnimation::new(start_idx, start_idx + num_leds - 1)
                    .with_slot(slot)
                    .with_color(rgbw)
                    .with_frame_rate(200.0 / period_secs),
            )
        })
    }

    /// Rainbow advancing slowly across the strip.
    pub fn rainbow(&self) -> CANdlePattern {
        self.rainbow_with_brightness(1.0)
    }

    /// Rainbow advancing slowly across the strip, at the given brightness, 0.0-1.0.
    pub fn rainbow_with_brightness(&self, brightness: f64) -> CANdlePattern {
        let slot = self.state.config.animation_slot;
        lazy_anim(move |start_idx, num_leds| {
            Box::new(
                RainbowAnimation::new(start_idx, start_idx + num_leds - 1)
                    .with_slot(slot)
                    .with_brightness(brightness)
                    .with_frame_rate(3.0),
            )
        })
    }

    /// Rainbow advancing quickly across the strip.
    pub fn scrolling_rainbow(&self) -> CANdlePattern {
        let slot = self.state.config.animation_slot;
        lazy_anim(move |start_idx, num_leds| {
            Box::new(
                RainbowAnimation::new(start_idx, start_idx + num_leds - 1)
                    .with_slot(slot)
                    .with_brightness(1.0)
                    .with_frame_rate(60.0),
            )
        })
    }

    /// Lights one LED at a time across the strip and repeats. The frame rate is `num_leds *
    /// speed`, so `speed` full cycles happen per second.
    pub fn chase(&self, color: Color, speed: f64) -> CANdlePattern {
        let rgbw = to_rgbw(color);
        let slot = self.state.config.animation_slot;
        lazy_anim(move |start_idx, num_leds| {
            Box::new(
                ColorFlowAnimation::new(start_idx, start_idx + num_leds - 1)
                    .with_slot(slot)
                    .with_color(rgbw)
                    .with_frame_rate(f64::from(num_leds) * speed),
            )
        })
    }

    /// Bouncing dot that travels back and forth along the strip. `duration_secs` is the
    /// seconds for one complete back-and-forth cycle.
    pub fn bounce(&self, color: Color, duration_secs: f64) -> CANdlePattern {
        let rgbw = to_rgbw(color);
        let slot = self.state.config.animation_slot;
        lazy_anim(move |start_idx, num_leds| {
            Box::new(
                LarsonAnimation::new(start_idx, start_idx + num_leds - 1)
                    .with_slot(slot)
                    .with_color(rgbw)
                    .with_size(3)
                    .with_bounce_mode(LarsonBounceValue::Back)
                    .with_frame_rate(bounce_frame_rate(num_leds, duration_secs)),
            )
        })
    }

    /// Fire animation from the CANdle's own animation engine.
    pub fn fire(&self) -> CANdlePattern {
        let slot = self.state.config.animation_slot;
        lazy_anim(move |start_idx, num_leds| {
            Box::new(
                FireAnimation::new(start_idx, start_idx + num_leds - 1)
                    .with_slot(slot)
                    .with_frame_rate(60.0),
            )
        })
    }

    /// RGB color cycle from the CANdle's own animation engine.
    pub fn rgb_cycle(&self) -> CANdlePattern {
        let slot = self.state.config.animation_slot;
        lazy_anim(move |start_idx, num_leds| {
            Box::new(
                RgbFadeAnimation::new(start_idx, start_idx + num_leds - 1)
                    .with_slot(slot)
                    .with_frame_rate(30.0),
            )
        })
    }

    /// Solid color, from a single `SolidColor` control request resent each loop.
    pub fn solid(&self, color: Color) -> CANdlePattern {
        let rgbw = to_rgbw(color);
        CANdlePattern::new(lazy(move |start_idx, num_leds| {
            Box::new(SolidColor::new(start_idx, start_idx + num_leds - 1).with_color(rgbw))
        }))
    }

    /// Two-color stripe: the first `percent` fraction of LEDs shows `color1` and the rest
    /// shows `color2`, sent as two `SolidColor` controls. `percent` is 0.0-1.0.
    pub fn stripe(&self, percent: f64, color1: Color, color2: Color) -> CANdlePattern {
        let rgbw1 = to_rgbw(color1);
        let rgbw2 = to_rgbw(color2);
        let holder: RefCell<Option<Vec<SolidColor>>> = RefCell::new(None);
        CANdlePattern::new(move |candle, start_idx, num_leds| {
            let mut holder = holder.borrow_mut();
            let requests = holder.get_or_insert_with(|| {
                let [first, second] = stripe_segments(start_idx, num_leds, percent);
                let mut requests = Vec::new();
                // Segment 1 may be empty, and so may segment 2.
                if let Some((start, end)) = first {
                    requests.push(SolidColor::new(start, end).with_color(rgbw1));
                }
                if let Some((start, end)) = second {
                    requests.push(SolidColor::new(start, end).with_color(rgbw2));
                }
                requests
            });
            for request in requests.iter() {
                candle.set_control(request);
            }
        })
    }

    /// Linear gradient from `color1` to `color2` across the segment, one `SolidColor` per
    /// LED, computed at first use.
    pub fn gradient(&self, color1: Color, color2: Color) -> CANdlePattern {
        let holder: RefCell<Option<Vec<SolidColor>>> = RefCell::new(None);
        CANdlePattern::new(move |candle, start_idx, num_leds| {
            let mut holder = holder.borrow_mut();
            let requests = holder.get_or_insert_with(|| {
                (0..num_leds)
                    .map(|i| {
                        SolidColor::new(start_idx + i, start_idx + i).with_color(lerp(
                            color1,
                            color2,
                            gradient_ratio(i, num_leds),
                        ))
                    })
                    .collect()
            });
            for request in requests.iter() {
                candle.set_control(request);
            }
        })
    }

    /// Lights the first and last `length` LEDs with `color` and turns the middle off, with
    /// two or three `SolidColor` controls.
    pub fn edges(&self, color: Color, length: i32) -> CANdlePattern {
        let rgbw = to_rgbw(color);
        let holder: RefCell<Option<Vec<SolidColor>>> = RefCell::new(None);
        CANdlePattern::new(move |candle, start_idx, num_leds| {
            let mut holder = holder.borrow_mut();
            let requests = holder.get_or_insert_with(|| {
                edge_segments(start_idx, num_leds, length)
                    .into_iter()
                    .map(|(start, end, lit)| {
                        SolidColor::new(start, end).with_color(if lit { rgbw } else { OFF })
                    })
                    .collect()
            });
            for request in requests.iter() {
                candle.set_control(request);
            }
        })
    }

    /// Ombre that blends between the two colors and scrolls the blend point along the strip
    /// over time. The colors are rebuilt every loop because `RgbwColor` is a plain value.
    pub fn ombre(&self, start_color: Color, end_color: Color) -> CANdlePattern {
        per_led(move |num_leds, colors| {
            let phase_shift = ombre_phase_shift(current_time_millis());
            for (i, color) in colors.iter_mut().enumerate() {
                *color = lerp(
                    start_color,
                    end_color,
                    ombre_ratio(i, num_leds, phase_shift),
                );
            }
        })
    }

    /// Sine wave blending between the two colors, one `SolidColor` per LED.
    ///
    /// `cycle_length` is the number of LEDs per wave period, `duration_secs` is the period of
    /// the time-based animation in seconds.
    pub fn wave(
        &self,
        c1: Color,
        c2: Color,
        cycle_length: f64,
        duration_secs: f64,
    ) -> CANdlePattern {
        let x_diff_per_led = (2.0 * std::f64::consts::PI) / cycle_length;
        per_led(move |_num_leds, colors| {
            let phase = (timer::fpga_timestamp() % duration_secs) / duration_secs;
            let x0 = (1.0 - phase) * 2.0 * std::f64::consts::PI;
            for (i, color) in colors.iter_mut().enumerate() {
                let x = x0 + (i as f64 + 1.0) * x_diff_per_led;
                *color = lerp(c1, c2, wave_ratio(x));
            }
        })
    }

    /// Counts down by turning LEDs off from the end of the segment toward the start, while
    /// the color fades from yellow to red.
    ///
    /// `count_start_time_sec` supplies the FPGA timestamp (seconds) when the countdown began.
    /// `duration_in_seconds` is the total countdown duration in seconds.
    pub fn countdown(
        &self,
        count_start_time_sec: impl Fn() -> f64 + 'static,
        duration_in_seconds: f64,
    ) -> CANdlePattern {
        per_led(move |num_leds, colors| {
            // The supplier is read each loop (not at factory time) so patterns built at
            // binding time still measure from the correct start when the command runs.
            let elapsed = timer::fpga_timestamp() - count_start_time_sec();
            let progress = (elapsed / duration_in_seconds).min(1.0);
            let on = countdown_color(progress);
            fill_countdown(colors, num_leds, progress, on);
        })
    }

    /// Alliance switch countdown. Colors follow a hard-coded match-time schedule, and LEDs
    /// turn off within each segment as its time runs out.
    ///
    /// Seconds remaining, and the color shown:
    ///
    /// ```text
    ///  140-130  purple
    ///  130-105  starting_color
    ///  105-80   opponent color
    ///   80-55   starting_color
    ///   55-30   opponent color
    ///   30-0    purple
    /// ```
    pub fn switch_countdown(&self, starting_color: Color) -> CANdlePattern {
        per_led(move |num_leds, colors| {
            let elapsed = 140.0 - timer::match_time();
            let (color, progress) = switch_countdown_segment(elapsed, starting_color);
            fill_countdown(colors, num_leds, progress, to_rgbw(color));
        })
    }
}

impl Subsystem for SpectrumLEDs {
    fn id(&self) -> SubsystemId {
        self.id
    }

    fn name(&self) -> String {
        "SpectrumLEDs".to_owned()
    }
}

fn set_pattern_command(
    id: SubsystemId,
    state: &Rc<State>,
    pattern: &CANdlePattern,
    priority: i32,
) -> Command {
    let (run_state, end_state, pattern) = (state.clone(), state.clone(), pattern.clone());
    Commands::run(
        move || {
            run_state.command_priority.set(priority);
            let is_anim = pattern.is_hardware_animation();
            // Clear this instance's animation slot once when transitioning to a software
            // pattern. Only our own slot is cleared so other instances sharing the same
            // CANdle keep their animations running.
            if run_state.last_was_animation.get() && !is_anim {
                run_state
                    .candle
                    .set_control(&EmptyAnimation::new(run_state.config.animation_slot));
            }
            run_state.last_was_animation.set(is_anim);
            pattern.apply_to(
                &run_state.candle,
                run_state.config.start_idx,
                run_state.config.num_leds,
            );
        },
        &[id],
    )
    .finally_do(move || end_state.command_priority.set(-1))
    .ignoring_disable(true)
    .with_name("LEDs.setPattern")
}

fn blink_pattern(animation_slot: i32, color: Color, on_time_secs: f64) -> CANdlePattern {
    let rgbw = to_rgbw(color);
    lazy_anim(move |start_idx, num_leds| {
        Box::new(
            StrobeAnimation::new(start_idx, start_idx + num_leds - 1)
                .with_slot(animation_slot)
                .with_color(rgbw)
                .with_frame_rate(1.0 / on_time_secs),
        )
    })
}

const OFF: RgbwColor = RgbwColor::new_rgbw(0, 0, 0, 0);

/// Converts a WPILib `Color` (0.0-1.0 double components) to a `RgbwColor` with `white` 0.
fn to_rgbw(color: Color) -> RgbwColor {
    RgbwColor::new_rgbw(
        (color.red * 255.0) as i32,
        (color.green * 255.0) as i32,
        (color.blue * 255.0) as i32,
        0,
    )
}

/// Blends `a` toward `b` by `ratio` (0 = a, 1 = b).
fn lerp(a: Color, b: Color, ratio: f64) -> RgbwColor {
    RgbwColor::new_rgbw(
        (a.red * 255.0 * (1.0 - ratio) + b.red * 255.0 * ratio) as i32,
        (a.green * 255.0 * (1.0 - ratio) + b.green * 255.0 * ratio) as i32,
        (a.blue * 255.0 * (1.0 - ratio) + b.blue * 255.0 * ratio) as i32,
        0,
    )
}

/// One full cycle is 2 * (num_leds - 1) LED-position advances.
fn bounce_frame_rate(num_leds: i32, duration_secs: f64) -> f64 {
    2.0 * f64::from((num_leds - 1).max(1)) / duration_secs
}

/// Java's `Math.round`, which rounds halves up, cast to int.
fn java_round(value: f64) -> i32 {
    (value + 0.5).floor() as i32
}

/// The inclusive LED ranges of the two stripe colors. Either is `None` when empty.
fn stripe_segments(start_idx: i32, num_leds: i32, percent: f64) -> [Option<(i32, i32)>; 2] {
    let split = java_round(f64::from(num_leds) * percent)
        .min(num_leds)
        .max(0);
    [
        (split > 0).then_some((start_idx, start_idx + split - 1)),
        (split < num_leds).then_some((start_idx + split, start_idx + num_leds - 1)),
    ]
}

/// Position of LED `i` along a gradient, 0 at the first LED and 1 at the last.
fn gradient_ratio(i: i32, num_leds: i32) -> f64 {
    if num_leds <= 1 {
        0.0
    } else {
        f64::from(i) / f64::from(num_leds - 1)
    }
}

/// The inclusive LED ranges of an edges pattern: left edge, right edge, and the dark middle
/// when there is one. The flag is true for a lit range.
fn edge_segments(start_idx: i32, num_leds: i32, length: i32) -> Vec<(i32, i32, bool)> {
    let clamped_len = length.min(num_leds / 2);
    if clamped_len == 0 {
        return Vec::new();
    }
    let center_start = start_idx + clamped_len;
    let center_end = start_idx + num_leds - clamped_len - 1;
    let mut segments = vec![
        (start_idx, start_idx + clamped_len - 1, true),
        (
            start_idx + num_leds - clamped_len,
            start_idx + num_leds - 1,
            true,
        ),
    ];
    if center_start <= center_end {
        segments.push((center_start, center_end, false));
    }
    segments
}

/// Speed: 0.58 strip-lengths per second.
fn ombre_phase_shift(millis: i64) -> f64 {
    (millis as f64 / 1000.0) * 0.58 % 1.0
}

fn ombre_ratio(i: usize, num_leds: usize, phase_shift: f64) -> f64 {
    ((i as f64 + num_leds as f64 * phase_shift) / num_leds as f64) % 1.0
}

fn current_time_millis() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_millis() as i64)
}

/// Blend ratio of a wave at phase `x`. A negative sine has no real power, so it falls back
/// to the mirrored half, then to the midpoint.
fn wave_ratio(x: f64) -> f64 {
    const WAVE_EXPONENT: f64 = 0.4;
    let mut ratio = (x.sin().powf(WAVE_EXPONENT) + 1.0) / 2.0;
    if ratio.is_nan() {
        ratio = (-(x + std::f64::consts::PI).sin().powf(WAVE_EXPONENT) + 1.0) / 2.0;
    }
    if ratio.is_nan() {
        ratio = 0.5;
    }
    ratio
}

/// Yellow fading to red as `progress` goes from 0 to 1.
fn countdown_color(progress: f64) -> RgbwColor {
    RgbwColor::new_rgbw(255, (255.0 * (1.0 - progress)) as i32, 0, 0)
}

/// Turns LEDs off from the end of the segment as `progress` goes from 0 to 1.
fn fill_countdown(colors: &mut [RgbwColor], num_leds: usize, progress: f64, on: RgbwColor) {
    let leds_off = (num_leds as f64 * progress.min(1.0)) as i32;
    for (i, color) in colors.iter_mut().enumerate() {
        *color = if num_leds as i32 - i as i32 <= leds_off {
            OFF
        } else {
            on
        };
    }
}

/// The segment of the switch countdown that `elapsed` seconds into the match falls in: its
/// color, and how far through it the match is.
fn switch_countdown_segment(elapsed: f64, starting_color: Color) -> (Color, f64) {
    let times = [10, 25, 25, 25, 25, 30];
    let mut shift_time = 0;
    let mut cumulative_time = 0;
    let mut color = Color::k_black();

    for (i, time) in times.into_iter().enumerate() {
        cumulative_time += time;
        if f64::from(cumulative_time) > elapsed {
            shift_time = time;
            color = match i {
                0 | 5 => Color::k_purple(),
                1 | 3 => starting_color,
                2 | 4 => {
                    if Color::k_red() == starting_color {
                        Color::k_blue()
                    } else {
                        Color::k_red()
                    }
                }
                _ => Color::k_black(),
            };
            break;
        }
    }

    let progress = 1.0 - (f64::from(cumulative_time) - elapsed) / f64::from(shift_time.max(1));
    (color, progress)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(expected: f64, actual: f64) {
        assert!((expected - actual).abs() < 1e-9, "{expected} vs {actual}");
    }

    fn rgb(red: i32, green: i32, blue: i32) -> RgbwColor {
        RgbwColor::new_rgbw(red, green, blue, 0)
    }

    #[test]
    fn config_for_an_owned_candle_skips_the_onboard_leds() {
        let config = Config::new("Leds", 1, 40, CanBus::new("*"));
        assert_eq!(config.name, "Leds");
        assert_eq!(config.device_id, 1);
        assert_eq!(config.num_leds, 40);
        assert_eq!(config.start_idx, 8);
        assert_eq!(config.can_bus, Some(CanBus::new("*")));
        assert!(config.shared_candle.is_none());
        assert!(config.attached);
        assert_eq!(config.animation_slot, 0);
        assert_eq!(config.strip_type, StripTypeValue::RGB);
        assert_eq!(config.brightness, 1.0);
        assert_eq!(
            config.loss_of_signal_behavior,
            LossOfSignalBehaviorValue::DisableLEDs
        );
    }

    #[test]
    fn color_conversion_truncates_and_keeps_white_off() {
        assert_eq!(to_rgbw(Color::k_white()), rgb(255, 255, 255));
        assert_eq!(to_rgbw(Color::k_black()), rgb(0, 0, 0));
        assert_eq!(to_rgbw(Color::from_rgb(130, 103, 185)), rgb(130, 103, 185));
        assert_eq!(to_rgbw(Color::k_orange()), rgb(255, 165, 0));
        assert_eq!(to_rgbw(Color::new(0.5, 0.5, 0.5)), rgb(127, 127, 127));
    }

    #[test]
    fn lerp_blends_from_a_to_b() {
        let (a, b) = (Color::k_red(), Color::k_blue());
        assert_eq!(lerp(a, b, 0.0), rgb(255, 0, 0));
        assert_eq!(lerp(a, b, 1.0), rgb(0, 0, 255));
        assert_eq!(lerp(a, b, 0.5), rgb(127, 0, 127));
    }

    #[test]
    fn bounce_frame_rate_makes_one_cycle_per_duration() {
        // 10 LEDs: 18 advances in 2 s is 9 frames per second.
        assert_close(9.0, bounce_frame_rate(10, 2.0));
        // A one-LED segment still advances once per half cycle.
        assert_close(1.0, bounce_frame_rate(1, 2.0));
        assert_close(1.0, bounce_frame_rate(0, 2.0));
    }

    #[test]
    fn java_round_rounds_halves_up() {
        assert_eq!(java_round(2.5), 3);
        assert_eq!(java_round(-2.5), -2);
        assert_eq!(java_round(2.4), 2);
        assert_eq!(java_round(-2.6), -3);
    }

    #[test]
    fn stripe_splits_the_segment_by_percent() {
        assert_eq!(stripe_segments(8, 10, 0.3), [Some((8, 10)), Some((11, 17))]);
        assert_eq!(stripe_segments(8, 10, 0.0), [None, Some((8, 17))]);
        assert_eq!(stripe_segments(8, 10, 1.0), [Some((8, 17)), None]);
        assert_eq!(stripe_segments(8, 10, 2.0), [Some((8, 17)), None]);
        assert_eq!(stripe_segments(8, 10, -1.0), [None, Some((8, 17))]);
        assert_eq!(stripe_segments(0, 10, 0.25), [Some((0, 2)), Some((3, 9))]);
    }

    #[test]
    fn gradient_runs_from_the_first_led_to_the_last() {
        assert_close(0.0, gradient_ratio(0, 5));
        assert_close(0.5, gradient_ratio(2, 5));
        assert_close(1.0, gradient_ratio(4, 5));
        assert_close(0.0, gradient_ratio(0, 1));
        assert_close(0.0, gradient_ratio(0, 0));
    }

    #[test]
    fn edges_light_both_ends_and_darken_the_middle() {
        assert_eq!(
            edge_segments(8, 10, 2),
            [(8, 9, true), (16, 17, true), (10, 15, false)]
        );
        assert_eq!(edge_segments(0, 10, 0), []);
        assert_eq!(edge_segments(0, 1, 5), []);
    }

    #[test]
    fn edges_that_meet_leave_no_dark_middle() {
        assert_eq!(edge_segments(8, 10, 5), [(8, 12, true), (13, 17, true)]);
        // The length clamps to half the segment.
        assert_eq!(edge_segments(8, 10, 50), [(8, 12, true), (13, 17, true)]);
    }

    #[test]
    fn ombre_phase_scrolls_at_point_58_strips_per_second() {
        assert_close(0.0, ombre_phase_shift(0));
        assert_close(0.58, ombre_phase_shift(1000));
        assert_close(0.16, ombre_phase_shift(2000));
        assert_close(0.29, ombre_phase_shift(500));
    }

    #[test]
    fn ombre_ratio_wraps_around_the_strip() {
        assert_close(0.0, ombre_ratio(0, 10, 0.0));
        assert_close(0.5, ombre_ratio(5, 10, 0.0));
        assert_close(0.3, ombre_ratio(0, 10, 0.3));
        assert_close(0.1, ombre_ratio(8, 10, 0.3));
    }

    #[test]
    fn wave_ratio_uses_the_mirrored_half_for_negative_sines() {
        // sin(pi/2) = 1 gives the top of the wave.
        assert_close(1.0, wave_ratio(std::f64::consts::FRAC_PI_2));
        // A negative sine would be NaN, so the mirrored half answers.
        let mirrored = wave_ratio(3.0 * std::f64::consts::FRAC_PI_2);
        assert!((mirrored - 0.0).abs() < 1e-9, "{mirrored}");
        assert!(!wave_ratio(4.0).is_nan());
        assert!((0.0..=1.0).contains(&wave_ratio(5.5)));
    }

    #[test]
    fn countdown_fades_from_yellow_to_red() {
        assert_eq!(countdown_color(0.0), rgb(255, 255, 0));
        assert_eq!(countdown_color(0.5), rgb(255, 127, 0));
        assert_eq!(countdown_color(1.0), rgb(255, 0, 0));
    }

    #[test]
    fn countdown_turns_leds_off_from_the_end() {
        let on = rgb(255, 100, 0);
        let mut colors = vec![OFF; 10];
        fill_countdown(&mut colors, 10, 0.0, on);
        assert!(colors.iter().all(|c| *c == on));

        fill_countdown(&mut colors, 10, 0.3, on);
        assert_eq!(colors.iter().filter(|c| **c == OFF).count(), 3);
        assert!(colors[..7].iter().all(|c| *c == on));
        assert!(colors[7..].iter().all(|c| *c == OFF));

        fill_countdown(&mut colors, 10, 1.0, on);
        assert!(colors.iter().all(|c| *c == OFF));
        fill_countdown(&mut colors, 10, 7.0, on);
        assert!(colors.iter().all(|c| *c == OFF));
    }

    #[test]
    fn switch_countdown_follows_the_match_schedule() {
        let blue = Color::k_blue();
        // 140-130 purple.
        assert_eq!(switch_countdown_segment(0.0, blue).0, Color::k_purple());
        // 130-105 starting color.
        assert_eq!(switch_countdown_segment(10.0, blue).0, blue);
        // 105-80 opponent color.
        assert_eq!(switch_countdown_segment(35.0, blue).0, Color::k_red());
        // 80-55 starting color again.
        assert_eq!(switch_countdown_segment(60.0, blue).0, blue);
        // 55-30 opponent color.
        assert_eq!(switch_countdown_segment(85.0, blue).0, Color::k_red());
        // 30-0 purple.
        assert_eq!(switch_countdown_segment(110.0, blue).0, Color::k_purple());
    }

    #[test]
    fn red_alliance_sees_blue_as_the_opponent() {
        let red = Color::k_red();
        assert_eq!(switch_countdown_segment(35.0, red).0, Color::k_blue());
        assert_eq!(switch_countdown_segment(85.0, red).0, Color::k_blue());
        assert_eq!(switch_countdown_segment(10.0, red).0, red);
    }

    #[test]
    fn switch_countdown_progress_runs_through_each_segment() {
        let blue = Color::k_blue();
        assert_close(0.0, switch_countdown_segment(0.0, blue).1);
        assert_close(0.5, switch_countdown_segment(5.0, blue).1);
        // The next segment starts over.
        assert_close(0.0, switch_countdown_segment(10.0, blue).1);
        assert_close(0.4, switch_countdown_segment(20.0, blue).1);
    }

    #[test]
    fn switch_countdown_past_the_end_turns_everything_off() {
        let (color, progress) = switch_countdown_segment(140.0, Color::k_blue());
        assert_eq!(color, Color::k_black());
        assert!(progress >= 1.0);
        // No match running reports -1 seconds left, which is 141 elapsed.
        let (color, progress) = switch_countdown_segment(141.0, Color::k_blue());
        assert_eq!(color, Color::k_black());
        let mut colors = vec![rgb(1, 1, 1); 4];
        fill_countdown(&mut colors, 4, progress, to_rgbw(color));
        assert!(colors.iter().all(|c| *c == OFF));
    }

    #[test]
    fn purple_is_the_spectrum_purple() {
        assert_eq!(
            Color::from_rgb(130, 103, 185),
            Color::from_rgb(130, 103, 185)
        );
        assert_eq!(to_rgbw(Color::from_rgb(130, 103, 185)), rgb(130, 103, 185));
    }
}
