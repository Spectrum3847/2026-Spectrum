package frc.robot.subsystems.leds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SuperStructure;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.leds.SpectrumLEDs;
import frc.spectrumLib.telemetry.Telemetry;

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

    /**
     * Static hardware config. Set {@code startIdx = 8} to address only the external strip (skipping
     * the 8 onboard CANdle LEDs); keep at {@code 0} to address all 20 LEDs starting from the first
     * onboard LED.
     */
    public static final Config ledsConfig;

    static {
        ledsConfig = new Config("Leds", 1, NUM_LEDS, new CANBus(Rio.CANIVORE));
        ledsConfig.setStripType(StripTypeValue.RGB);
        ledsConfig.setBrightness(0.5);
        ledsConfig.setLossOfSignalBehavior(LossOfSignalBehaviorValue.DisableLEDs);
    }

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    private static SuperStructure robotSuperStructure;

    public Leds(SuperStructure superStructure) {
        super(ledsConfig);

        robotSuperStructure = superStructure;
        setDefaultCommand(setPattern(breathe(purple, 2.0), -1).withName("Leds.idle"));

        Telemetry.print(getName() + " Subsystem Initialized");
    }

    // TODO: add more patterns and include ones for alliance shifts and bps

    static void bindTriggers() {
        launchingLed(launchingFuel, 5);
        blueShiftLed(blueShift, 3);
        redShiftLed(redShift, 3);
    }

    private static Trigger redShfit = 
            new Trigger(
                    () ->   {
                                double t = DriverStation.getMatchTime();
                                ShiftHelpers.getFirstActiveAlliance
                                        == Alliance.Red && (t >= 10.0 && t <= 35.0) ||
                                            (t >= 60.0 && t <= 85.0);
                            })
                    .and(Utils.teleop);

    private static Trigger blueShfit =             
            new Trigger(
                    () ->   {
                                double t = DriverStation.getMatchTime();
                                ShiftHelpers.getFirstActiveAlliance
                                        == Alliance.Blue && (t >= 10.0 && t <= 35.0) ||
                                            (t >= 60.0 && t <= 85.0);
                            })
                    .and(Utils.teleop);
    

    private static Trigger launchingFuel =
            new Trigger(
                    () ->
                            robotSuperStructure.getCurrentSuperState()
                                    == SuperStructure.CurrentSuperState.LAUNCH_WITH_SQUEEZE
            );

    static void redShiftLed(Trigger trigger, int priority) {
        ledCommand("Leds.Red_Shift", bounce('red', 25), priority, trigger);
    }

    static void blueShiftLed(Trigger trigger, int priority) {
        ledCommand("Leds.Blue_Shift", bounce('blue', 25), priority, trigger);
    }

    static void launchingLed(Trigger trigger, int priority) {
        ledCommand("Leds.launching", rainbow(0.5), priority, trigger);
    }

    private static Trigger ledCommand(String name, CANdlePattern pattern, int priority, Trigger trigger) {
        return trigger.and(checkPriority(priority))
                .whileTrue(setPattern(pattern, priority).withName(name));
    }

    @Override
    public void periodic() {
        Telemetry.log("Leds/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Leds/CommandPriority", getCommandPriority());
        Telemetry.log("Leds/IsAnimating", isAnimating());
    }
}
