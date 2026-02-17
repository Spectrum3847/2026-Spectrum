package frc.robot.leds;

import frc.robot.Robot;
import frc.robot.pilot.PilotStates;
import frc.spectrumLib.leds.SpectrumLEDs;
import frc.spectrumLib.util.Util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

public class LedCANdle extends LedStates{
    ;
    private final CANdle candle = new CANdle(0);

    private static LedFull leds = Robot.getLeds();
    private static LedRight right = leds.getRight();
    private static LedLeft left = leds.getLeft();

    private enum AnimationType {
        None,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
    }

    public LedCANdle() {

        CANdleConfiguration ledConfig = new CANdleConfiguration();
        // ledConfig.LED.stripType = StripTypeValue.;
        ledConfig.LED.BrightnessScalar = 0.5;

        candle.getConfigurator().apply(ledConfig);

        static void test(Trigger trigger, int priority) {
            ledCommand(
                "right.testPattern", right, right.SingleFade(right.purple), priority, trigger
            );
            ledCommand(
                "left.testPattern", left, left.SingleFade(left.purple), priority, trigger
            );
        }

    }

}
