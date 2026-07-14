package frc.robot.subsystems.leds;

import com.ctre.phoenix6.CANBus;
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
 * breathe, rainbow, chase, bounce, gradient, ombre, wave, countdown, etc.) and the AddressableLED
 * hardware abstraction. Robot-specific convenience command methods are defined below; bind them via
 * triggers in {@code Robot.java} or {@code SuperStructure}.
 *
 * <p>Hardware: PWM port 0, 20-LED RGB external strip.
 */
public class Leds extends SpectrumLEDs {

    // -------------------------------------------------------------------------
    // Hardware configuration
    // -------------------------------------------------------------------------

    /** Number of external LEDs attached to the roboRIO PWM port. */
    public static final int NUM_LEDS = 20;

    public static class LedConfig extends CANdleConfig {
        public LedConfig() {
            super("Leds", 1, NUM_LEDS, new CANBus(Rio.CANIVORE));
        }

        public LedConfig(
                String name,
                edu.wpi.first.wpilibj.AddressableLED l,
                edu.wpi.first.wpilibj.AddressableLEDBuffer lb,
                int startingIndex,
                int endingIndex) {
            super(name, 1, NUM_LEDS, new CANBus(Rio.CANIVORE));
        }
    }

    @Getter @Setter private LedConfig config;

    // -------------------------------------------------------------------------
    // Pattern state
    // -------------------------------------------------------------------------

    /** Default pattern shown when no command is running. */
    private final CANdlePattern defaultPattern = breathe(purple, 2.0);

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    public Leds(LedConfig config) {
        super(config);

        this.config = config;
        setDefaultCommand(setPattern(defaultPattern, -1).withName("Leds.idle"));

        bindTriggers();

        Telemetry.print(getName() + " Subsystem Initialized");
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

    private Trigger autonomous = new Trigger(() -> DriverStation.isAutonomous());

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
}
