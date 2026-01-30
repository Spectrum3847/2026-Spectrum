package frc.robot.leds;

import frc.spectrumLib.leds.SpectrumLEDs2;
import lombok.Getter;

public class LedFull extends SpectrumLEDs2 {

    public static class LedFullConfig extends Config {
        public LedFullConfig() {
            super("LEDS", 30 * 2, 61); // 60 LEDs total, 30 per side. CAN ID 61.
        }
    }

    protected LedFullConfig config;
    @Getter
    protected LedRight right;
    @Getter
    protected LedLeft left;

    public LedFull(LedFullConfig config) {
        super(config);
        this.config = config;

        // Initialize segments sharing the same CANdle instance
        // Right side is 0-29? Left side is 30-59?
        // Based on original LedRight: 0 to length/2 - 1
        // LedLeft: length/2 to length - 1

        int halfLength = config.getLength() / 2;

        // Pass the CANdle instance from this (LedFull) to the children
        right = new LedRight(new LedRight.LedConfig(getLeds(), halfLength, 0));
        left = new LedLeft(new LedLeft.LedConfig(getLeds(), halfLength, halfLength));
    }

    @Override
    public void setupStates() {
        LedStates.bindTriggers();
    }

    @Override
    public void setupDefaultCommand() {
        setDefaultCommand(defaultCommand);
    }
}
