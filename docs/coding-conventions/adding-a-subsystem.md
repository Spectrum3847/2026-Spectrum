# Adding a subsystem

*Audience: reference. Assumes you have read [Code style](code-style.md).*

This is the layout every mechanism follows. When you are not sure how to build something, copy `hood.rs`. It is the shortest.

## One file per mechanism

A mechanism is one file in `rust/crates/robot/src/robot/subsystems/`. The file holds:

```text
hood.rs
  HoodConfig        every tunable value, wrapping a mechanism Config
  WantedState       what a caller asked for
  SystemState       what the mechanism is doing about it
  handle_state_transition(wanted) -> SystemState
  output_for(system_state, ...) -> Output
  Hood              the subsystem
```

`HoodConfig` has a `base: Config` field (`spectrum_lib/mechanism/mechanism.rs`) with the motor ID, bus, gains, limits and gear ratio, plus the mechanism's own numbers. The `Default` impl sets all of them. `base.attached` says if the hardware exists.

`Hood` holds an `Rc<Mechanism>`, the wanted and system states in `Cell`s, and its config. It implements `Deref<Target = Mechanism>`, so you can call mechanism methods on it, and it implements the `Subsystem` trait.

The state logic is small pure functions (`handle_state_transition`, `output_for`), so tests run them with no hardware.

## How it runs each loop

The scheduler calls `Subsystem::periodic` on every registered subsystem each loop. In `periodic`:

1. `handle_state_transition` turns the wanted state into the system state.
2. `apply_states` turns the system state into a motor request.
3. The subsystem logs its states, voltages and currents with `telemetry::log_dash`.

A new wanted state therefore takes effect on the loop after the one that set it.

## The orchestrator

`SuperStructure` (`super_structure.rs`) sits above the mechanisms. `set_wanted_super_state` picks one robot-level state. `plan_for` maps each `CurrentSuperState` to a `Plan`, which holds the wanted state of every mechanism. `apply_states` sends those to the mechanisms. Gamepad bindings and `Auton` talk to `SuperStructure`, never to a mechanism. `set_state_command` wraps a state change as a command for bindings.

## Steps to add a mechanism

1. Create `subsystems/<name>.rs` and add `pub mod <name>;` in `subsystems/mod.rs`.
2. Write the config struct, with `Config::new("<Name>", can_id, bus)` and the `config_*` calls for gains, limits, gear ratio and current limits. Put CAN IDs here. Put per-robot differences in the robot configs, not here.
3. Write `WantedState` and `SystemState`, with `EnumName` so logs show readable names.
4. Write the `Subsystem` impl. Register it with `CommandScheduler::get_instance().register_subsystem(...)` in its constructor.
5. Add a field for its config to `Config` in `robot/robot.rs`.
6. In `robot/configs/`, set `<name>.base.attached = false` for each robot without the hardware. The default is `true`. An unattached mechanism builds no hardware and reads zero.
7. Build it in `Robot::init` in `robot.rs`, in the same order and with the same `timer::delay(can_init_delay)` as the others. If other code reads it, call `context::set_<name>` there too, and declare it in `context.rs`.
8. Pass it to `SuperStructure::new`, add its wanted state to `Plan`, and fill it in for each state in `plan_for`.
9. Add tests: the transition table, the output for each state, and that an unattached mechanism commands nothing.
10. Run `cargo xtask gate` and the [simulation](../tools/simulation.md).

For a new robot state, see `WantedSuperState` and `CurrentSuperState` in `super_structure.rs`. Add the variant to both enums and to `EnumName`, `handle_state_transitions` and `plan_for`. Add a binding in `bind_controls` (`robot.rs`) if a button starts it.

## Things to know

* **No constructor overloads.** Rust has none. One `new` takes the config. Use a builder method (`with_...`) for optional parts.
* **Pass functions for live setpoints.** Mechanism methods such as `set_mm_position_foc` take a closure (`impl Fn() -> f64`) instead of a `f64`. The closure runs when the request is sent. A plain number freezes the value at that moment.
* **Triggers.** Return a `Trigger` for a condition that starts commands (`at_rotations`, `above_velocity_rpm` and others on `Mechanism`). Triggers allocate, so build them at bind time and not every loop.
* **No allocation in `periodic`.** It runs 50 times a second on a small computer. Use plain loops. Avoid `format!` and `Vec` in code that runs every loop.
* **Booleans as parameters.** A method with a `reverse: bool` is usually two methods with clearer names.
* **Keep methods small.** A method that both computes a setpoint and drives the motor is hard to test. Compute in a pure function and apply in another.

## Documentation

Every `pub` item that other code uses gets a doc comment. Config fields get their unit. See [Documentation and comments](documentation-and-comments.md).
