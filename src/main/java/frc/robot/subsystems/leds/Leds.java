package frc.robot.subsystems.leds;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.rebuilt.ShiftHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.SuperStructure;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.leds.SpectrumLEDs;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.util.Util;
import lombok.Getter;
import lombok.Setter;

/**
 * Robot LED subsystem for the 2026 REBUILT season.
 *
 * <p>Extends {@link SpectrumLEDs} to inherit the full pattern library (solid, stripe, blink,
 * breathe, rainbow, chase, bounce, gradient, ombre, wave, countdown, etc.) and the CANdle hardware
 * abstraction. Robot-specific convenience command methods are defined below; bind them via triggers
 * in {@code Robot.java} or {@code SuperStructure}.
 *
 * <p>Hardware: CANdle device ID 1 on the CANivore bus, 20-LED RGB external strip at brightness 0.5.
 * LEDs are disabled on signal loss.
 */
public class Leds extends SpectrumLEDs {

    // -------------------------------------------------------------------------
    // Hardware configuration
    // -------------------------------------------------------------------------

    /** Number of external LEDs attached to the CANdle output (indices 8–27 on the device). */
    public static final int NUM_LEDS = 20;

    @Getter @Setter protected AddressableLED led;
    @Getter @Setter protected AddressableLEDBuffer buffer;
    @Getter @Setter protected AddressableLEDBufferView view;

    public static class LedConfig extends CANdleConfig {
        @Getter protected String name;
        @Getter @Setter protected boolean attached = true;
        @Getter @Setter protected int startingIndex = 0;
        @Getter @Setter protected int endingIndex = 28;
        @Getter @Setter protected int port = 0;

        public LedConfig(
                String name,
                AddressableLED l,
                AddressableLEDBuffer lb,
                int startingIndex,
                int endingIndex) {
            super("Leds", 1, NUM_LEDS, new CANBus(Rio.RIO_CANBUS));
            this.name = name;
            this.led = l;
            this.buffer = lb;
            this.startingIndex = startingIndex;
            this.endingIndex = endingIndex;
        }
    }

    @Getter protected LedConfig config;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    public Leds(LedConfig config) {
        super(config);

        this.config = config;
        setDefaultCommand(setPattern(breathe(purple, 2.0), -1).withName("Leds.idle"));

        bindTriggers();

        Telemetry.print(getName() + " Subsystem Initialized");

        if (config.getLed() == null) {
                led = new AddressableLED(config.port);
                // Length is expensive to set, so only set it once, then just update data
                ledBuffer = new AddressableLEDBuffer(NUM_LEDS);
                led.setLength(ledBuffer.getLength());
                mainView = true;
            } else {
                led = config.getLed();
                ledBuffer = config.buffer;
            }

            ledView = ledBuffer.createView(config.startingIndex, config.endingIndex);

            // Set the data
            led.setData(ledBuffer);
            setPattern(defaultPattern);
            led.start();
    }

    // TODO: add more patterns and include ones for bps

    void bindTriggers() {
        launchingLed(launchingFuel, 1);
        redShiftLed(redShfit, 2);
        blueShiftLed(blueShfit, 2);
        bothHubsActiveLed(bothHubsActive, 2);
        autonLed(autonomouns, 2);
    }

    private Trigger redShfit =
            new Trigger(
                            () -> {
                                double t = DriverStation.getMatchTime();
                                return ShiftHelpers.getFirstActiveAlliance() == Alliance.Red
                                        && ((t >= 11.0 && t <= 35.0) || (t >= 61.0 && t <= 85.0));
                            })
                    .and(Util.teleop);

    private Trigger blueShfit =
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

    private Trigger autonomouns =
            new Trigger(
                    () -> {
                        return DriverStation.isAutonomous();
                    });

    // private Trigger bpsLow = new Trigger(() -> {})

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

    void redShiftLed(Trigger trigger, int priority) {
        ledCommand("Leds.Red_Shift", switchCountdown(Color.kRed), priority, trigger);
    }

    void blueShiftLed(Trigger trigger, int priority) {
        ledCommand("Leds.Blue_Shift", switchCountdown(Color.kBlue), priority, trigger);
    }

    void bothHubsActiveLed(Trigger trigger, int priority) {
        ledCommand("Leds.Both_Hubs_Active", gradient(Color.kBlue, Color.kRed), priority, trigger);
    }

    void autonLed(Trigger trigger, int priority) {
        ledCommand("Leds.Auton", countdown(() -> 0.0, 20.0), priority, trigger);
    }

    void launchingLed(Trigger trigger, int priority) {
        ledCommand("Leds.Launching", rainbow(0.5), priority, trigger);
    }

    private Trigger ledCommand(String name, CANdlePattern pattern, int priority, Trigger trigger) {
        return trigger.and(checkPriority(priority))
                .whileTrue(setPattern(pattern, priority).withName(name));
    }

    @Override
    public void periodic() {
        Telemetry.log("Leds/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Leds/CommandPriority", getCommandPriority());
        Telemetry.log("Leds/IsAnimating", isAnimating());

        if (mainView) {
            led.setData(ledBuffer);
        }
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------

        @SuppressWarnings("unused")
        private boolean mainView = false;

            // Must be a PWM header, not MXP or DIO
            if (config.getLed() == null) {
                led = new AddressableLED(config.port);
                // Length is expensive to set, so only set it once, then just update data
                ledBuffer = new AddressableLEDBuffer(NUM_LEDS);
                led.setLength(ledBuffer.getLength());
                mainView = true;
            } else {
                led = config.getLed();
                ledBuffer = config.buffer;
            }

            ledView = ledBuffer.createView(config.startingIndex, config.endingIndex);

            // Set the data
            led.setData(ledBuffer);
            setPattern(defaultPattern);
            led.start();
}

/*
 *     @Getter protected final AddressableLED led;
    @Getter protected final AddressableLEDBuffer ledBuffer;
    @Getter protected final AddressableLEDBufferView ledView;
    private boolean mainView = false;

    protected final LEDPattern defaultPattern = blink(Color.kOrange, 1);

    @Getter
    protected Command defaultCommand =
            setPattern(defaultPattern, -1).withName("LEDs.defaultCommand");

    public final Trigger defaultTrigger = new Trigger(() -> defaultCommand.isScheduled());

    @Getter @Setter private int commandPriority = 0;

    public final Color purple = new Color(130, 103, 185);
    public final Color white = Color.kWhite;

    public SpectrumLEDs(Config config) {
        this.config = config;

        // Must be a PWM header, not MXP or DIO
        if (config.getLed() == null) {
            led = new AddressableLED(config.port);
            // Length is expensive to set, so only set it once, then just update data
            ledBuffer = new AddressableLEDBuffer(config.length);
            led.setLength(ledBuffer.getLength());
            mainView = true;
        } else {
            led = config.getLed();
            ledBuffer = config.buffer;
        }

        ledView = ledBuffer.createView(config.startingIndex, config.endingIndex);

        // Set the data
        led.setData(ledBuffer);
        setPattern(defaultPattern);
        led.start();
 */