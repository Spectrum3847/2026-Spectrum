package frc.robot.leds;

import com.ctre.phoenix6.hardware.CANdle;
import frc.spectrumLib.leds.SpectrumLEDs2;

public class LedRight extends SpectrumLEDs2 {

    public static class LedConfig extends Config {
        public LedConfig(CANdle candle, int length, int startingIndex) {
            super("LEDS Right", length, candle, startingIndex);
        }
    }

    protected LedConfig config;

    public LedRight(LedConfig config) {
        super(config);
        this.config = config;
    }

    @Override
    public void setupStates() {
        // LedStates.bindTriggers();
    }

    @Override
    public void setupDefaultCommand() {
        setDefaultCommand(defaultCommand);
    }
}
