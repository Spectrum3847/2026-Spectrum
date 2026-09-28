package frc.robot.subsystems.leds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.leds.SpectrumLEDs;
import frc.spectrumLib.telemetry.Telemetry;

/**
 * LED strip for the robot. Extends {@link SpectrumLEDs} for the pattern library and the CANdle
 * hardware. Bind the pattern methods from {@code Robot.java} or {@code SuperStructure}.
 */
public class Leds extends SpectrumLEDs {

    /** Length of the external LED strip, which the CANdle addresses from index 8. */
    public static final int NUM_LEDS = 20;

    /**
     * Static hardware config, already set to address the external strip only. Set startIdx to 0 to
     * span the onboard LEDs as well.
     */
    public static final Config ledsConfig;

    static {
        ledsConfig = new Config("Leds", 1, NUM_LEDS, new CANBus(Rio.CANIVORE));
        ledsConfig.setStripType(StripTypeValue.RGB);
        ledsConfig.setBrightness(0.5);
        ledsConfig.setLossOfSignalBehavior(LossOfSignalBehaviorValue.DisableLEDs);
    }

    public Leds() {
        super(ledsConfig);

        setDefaultCommand(setPattern(breathe(purple, 2.0), -1).withName("Leds.idle"));

        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {
        Telemetry.log("Leds/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Leds/CommandPriority", getCommandPriority());
        Telemetry.log("Leds/IsAnimating", isAnimating());
    }
}
