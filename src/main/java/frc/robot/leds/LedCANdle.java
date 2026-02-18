package frc.robot.leds;

import frc.robot.Robot;
import frc.robot.pilot.PilotStates;
import frc.spectrumLib.leds.SpectrumLEDs;
import frc.spectrumLib.util.Util;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

public class LedCANdle extends LedStates{
    
    private final CANdle candle = new CANdle(0);

    private static LedFull leds = Robot.getLeds();
    private static LedRight right = leds.getRight();
    private static LedLeft left = leds.getLeft();

    private static final int kSlot0StartIdx = 8;
    private static final int kSlot0EndIdx = 37;

    private static final int kSlot1StartIdx = 38;
    private static final int kSlot1EndIdx = 67;

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

    private AnimationType animState0 = AnimationType.None;
    private AnimationType animState1 = AnimationType.None;

    private final SendableChooser<AnimationType> animChooser0 = new SendableChooser<AnimationType>();
    private final SendableChooser<AnimationType> animChooser1 = new SendableChooser<AnimationType>();

    public LedCANdle() {

        CANdleConfiguration ledConfig = new CANdleConfiguration();
        ledConfig.LED.StripType = StripTypeValue.RGBW;
        ledConfig.LED.BrightnessScalar = 0.5;

        candle.getConfigurator().apply(ledConfig);

        for (int i = 0; i < 8; i++) {
            candle.setControl(new EmptyAnimation(i));
        }

        animChooser0.setDefaultOption("Color Flow", AnimationType.ColorFlow);

        animChooser1.setDefaultOption("Fire", AnimationType.Fire);

        SmartDashboard.putData("Animation 0", animChooser0);
        SmartDashboard.putData("Animation 1", animChooser1);

    }

    void bind() {
        test(Util.teleop, 5);
    }

    void test(Trigger trigger, int priority) {
        animChooser0.setDefaultOption(
            "Color Flow Right", AnimationType.ColorFlow
        );
        animChooser1.addOption(
            "Color Flow Left", AnimationType.ColorFlow
        );
    }

    public void stateChanger() {
        /* if the selection for slot 0 changes, change animations */
        final var anim0Selection = animChooser0.getSelected();
        if (animState0 != anim0Selection) {
            animState0 = anim0Selection;

            switch (animState0) {
                default:
                case ColorFlow:
                    candle.setControl(
                        new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                    );
                    break;
                case Rainbow:
                    candle.setControl(
                        new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                    );
                    break;
                case Twinkle:
                    candle.setControl(
                        new TwinkleAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                    );
                    break;
                case TwinkleOff:
                    candle.setControl(
                        new TwinkleOffAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                    );
                    break;
                case Fire:
                    candle.setControl(
                        new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                    );
                    break;
            }
        }

        /* if the selection for slot 1 changes, change animations */
        final var anim1Selection = animChooser1.getSelected();
        if (animState1 != anim1Selection) {
            animState1 = anim1Selection;

            switch (animState1) {
                default:
                case Larson:
                    candle.setControl(
                        new LarsonAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                    );
                    break;
                case RgbFade:
                    candle.setControl(
                        new RgbFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                    );
                    break;
                case SingleFade:
                    candle.setControl(
                        new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                    );
                    break;
                case Strobe:
                    candle.setControl(
                        new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                    );
                    break;
                case Fire:
                    /* direction can be reversed by either the Direction parameter or switching start and end */
                    candle.setControl(
                        new FireAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                            .withDirection(AnimationDirectionValue.Backward)
                            .withCooling(0.4)
                            .withSparking(0.5)
                    );
                    break;
            }
        }
    }
}