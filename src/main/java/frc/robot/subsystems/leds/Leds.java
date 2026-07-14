package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.rebuilt.ShiftHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.SuperStructure;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.util.Util;
import lombok.Getter;
import lombok.Setter;

/**
 * Robot LED subsystem for the 2026 REBUILT season.
 *
 * <p>Uses WPILib {@link AddressableLED} to drive a WS2812B-compatible LED strip connected to a
 * roboRIO PWM port. Patterns are built using the WPILib {@link LEDPattern} API.
 *
 * <p>Hardware: PWM port 0, 20-LED RGB strip.
 */
public class Leds extends SubsystemBase {

    // -------------------------------------------------------------------------
    // Hardware configuration
    // -------------------------------------------------------------------------

    /** PWM port on the roboRIO (must be a PWM header, not MXP or DIO). */
    public static final int LED_PORT = 0;

    /** Number of LEDs on the strip. */
    public static final int NUM_LEDS = 20;

    @Getter @Setter private AddressableLED led;
    @Getter @Setter private AddressableLEDBuffer ledBuffer;
    @Getter @Setter private AddressableLEDBufferView ledView;

    /** True when this instance owns the LED hardware (created it, not shared). */
    private boolean mainView = false;

    // -------------------------------------------------------------------------
    // Pattern state
    // -------------------------------------------------------------------------

    @Getter @Setter private int commandPriority = -1;
    private LEDPattern currentPattern;

    /** Spectrum purple (RGB 130, 103, 185). */
    public static final Color purple = new Color(130 / 255.0, 103 / 255.0, 185 / 255.0);

    /** Default pattern shown when no command is running. */
    private final LEDPattern defaultPattern = LEDPattern.solid(purple)
            .breathe(Seconds.of(2.0));

    // -------------------------------------------------------------------------
    // Config
    // -------------------------------------------------------------------------

    public static class LedConfig {
        @Getter private String name;
        @Getter @Setter private int port = LED_PORT;
        @Getter @Setter private int length = NUM_LEDS;
        @Getter @Setter private int startingIndex = 0;
        @Getter @Setter private int endingIndex = NUM_LEDS - 1;
        @Getter @Setter private AddressableLED sharedLed = null;
        @Getter @Setter private AddressableLEDBuffer sharedBuffer = null;

        public LedConfig(String name) {
            this.name = name;
        }

        public LedConfig(String name, int port, int length) {
            this.name = name;
            this.port = port;
            this.length = length;
        }

        public LedConfig(String name, AddressableLED sharedLed, AddressableLEDBuffer sharedBuffer,
                         int startingIndex, int endingIndex) {
            this.name = name;
            this.sharedLed = sharedLed;
            this.sharedBuffer = sharedBuffer;
            this.startingIndex = startingIndex;
            this.endingIndex = endingIndex;
        }
    }

    @Getter private LedConfig config;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    public Leds(LedConfig config) {
        this.config = config;

        if (config.getSharedLed() == null) {
            // Own the hardware
            led = new AddressableLED(config.getPort());
            ledBuffer = new AddressableLEDBuffer(config.getLength());
            led.setLength(ledBuffer.getLength());
            mainView = true;
        } else {
            // Share an existing LED + buffer (multi-zone setup)
            led = config.getSharedLed();
            ledBuffer = config.getSharedBuffer();
            mainView = false;
        }

        ledView = ledBuffer.createView(config.getStartingIndex(), config.getEndingIndex());

        // Set initial data and start
        led.writeData(ledBuffer);
        led.start();

        // Set default pattern
        setDefaultCommand(setPattern(defaultPattern, -1).withName("Leds.idle"));

        // Bind triggers
        bindTriggers();

        Telemetry.print(getName() + " Subsystem Initialized");
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        Telemetry.log("Leds/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Leds/CommandPriority", getCommandPriority());

        if (mainView) {
            led.writeData(ledBuffer);
        }
    }

    // -------------------------------------------------------------------------
    // Pattern system
    // -------------------------------------------------------------------------

    /**
     * Returns a command that continuously applies {@code pattern} and tracks {@code priority}.
     * The command runs while the robot is disabled.
     *
     * @param pattern the {@link LEDPattern} to apply to the LED buffer view
     * @param priority priority level (higher = overrides lower)
     * @return a command that applies the pattern continuously
     */
    public Command setPattern(LEDPattern pattern, int priority) {
        return run(() -> {
                    commandPriority = priority;
                    currentPattern = pattern;
                    pattern.applyTo(ledView);
                })
                .finallyDo(() -> commandPriority = -1)
                .ignoringDisable(true)
                .withName("Leds.setPattern");
    }

    /** Convenience: set pattern at default priority 0. */
    public Command setPattern(LEDPattern pattern) {
        return setPattern(pattern, 0);
    }

    /** Returns a trigger that is active when the current command priority is at or below {@code priority}. */
    public Trigger checkPriority(int priority) {
        return new Trigger(() -> commandPriority <= priority);
    }

    // -------------------------------------------------------------------------
    // Triggers and bindings
    // -------------------------------------------------------------------------

    void bindTriggers() {
        launchingLed(launchingFuel, 1);
        redShiftLed(redShift, 2);
        blueShiftLed(blueShift, 2);
        bothHubsActiveLed(bothHubsActive, 2);
        autonLed(autonomous, 2);
    }

    private Trigger redShift =
            new Trigger(
                            () -> {
                                double t = DriverStation.getMatchTime();
                                return ShiftHelpers.getFirstActiveAlliance() == Alliance.Red
                                        && ((t >= 11.0 && t <= 35.0) || (t >= 61.0 && t <= 85.0));
                            })
                    .and(Util.teleop);

    private Trigger blueShift =
            new Trigger(
                            () -> {
                                double t = DriverStation.getMatchTime();
                                return ShiftHelpers.getFirstActiveAlliance() == Alliance.Blue
                                        && ((t >= 11.0 && t <= 35.0) || (t >= 61.0 && t <= 85.0));
                            })
                    .and(Util.teleop);

    private Trigger bothHubsActive =
            new Trigger(
                            () -> {
                                double t = DriverStation.getMatchTime();
                                return (t >= 0.0 && t <= 10.0) || (t >= 111.0 && t <= 140.0);
                            })
                    .and(Util.teleop);

    private Trigger autonomous =
            new Trigger(() -> DriverStation.isAutonomous());

    private Trigger launchingFuel =
            new Trigger(
                    () ->
                            Robot.getSuperStructure().getCurrentSuperState()
                                            == SuperStructure.CurrentSuperState.LAUNCH_WITH_SQUEEZE
                                    || Robot.getSuperStructure().getCurrentSuperState()
                                            == SuperStructure.CurrentSuperState
                                                    .LAUNCH_WITHOUT_SQUEEZE
                                    || Robot.getSuperStructure().getCurrentSuperState()
                                            == SuperStructure.CurrentSuperState
                                                    .LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY);

    // -------------------------------------------------------------------------
    // Pattern bindings
    // -------------------------------------------------------------------------

    void redShiftLed(Trigger trigger, int priority) {
        ledCommand("Leds.Red_Shift", LEDPattern.solid(Color.kRed), priority, trigger);
    }

    void blueShiftLed(Trigger trigger, int priority) {
        ledCommand("Leds.Blue_Shift", LEDPattern.solid(Color.kBlue), priority, trigger);
    }

    void bothHubsActiveLed(Trigger trigger, int priority) {
        // Gradient from blue to red across the strip
        ledCommand("Leds.Both_Hubs_Active",
                LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlue, Color.kRed),
                priority, trigger);
    }

    void autonLed(Trigger trigger, int priority) {
        // Rainbow during autonomous
        ledCommand("Leds.Auton", LEDPattern.rainbow(), priority, trigger);
    }

    void launchingLed(Trigger trigger, int priority) {
        // Rainbow when launching fuel
        ledCommand("Leds.Launching", LEDPattern.rainbow(), priority, trigger);
    }

    private Trigger ledCommand(String name, LEDPattern pattern, int priority, Trigger trigger) {
        return trigger.and(checkPriority(priority))
                .whileTrue(setPattern(pattern, priority).withName(name));
    }
}
