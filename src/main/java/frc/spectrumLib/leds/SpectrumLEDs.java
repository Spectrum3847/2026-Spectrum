package frc.spectrumLib.leds;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

/**
 * Addressable LED subsystem over a CTRE {@link CANdle}. Firmware patterns ({@link #blink}, {@link
 * #rainbow}, {@link #chase}, and the rest) hand the CANdle an animation and let the board render
 * it. Software patterns ({@link #solid}, {@link #gradient}, {@link #ombre}, and the rest) write
 * colors from robot code, resending a control request every loop.
 *
 * <p>Switching from a firmware pattern to a software pattern clears this instance's animation slot
 * so the board stops animating.
 *
 * <p>Several instances can share one physical {@link CANdle} by passing the same device to their
 * {@link Config} along with non-overlapping LED ranges and distinct animation slots.
 *
 * <p>{@link #setPattern(CANdlePattern, int)} returns a {@link Command} that runs continuously,
 * keeps running while the robot is disabled, and enforces {@link #checkPriority(int)}.
 */
public class SpectrumLEDs implements Subsystem {

    /**
     * Writes a pattern to a range of LEDs on a {@link CANdle}. {@link #setPattern} calls it once
     * per robot loop with the device and this instance's LED range.
     */
    @FunctionalInterface
    public interface CANdlePattern {
        void applyTo(CANdle candle, int startIdx, int numLeds);
    }

    /**
     * Marks a pattern as a firmware animation, so {@link #setPattern} can tell when a pattern
     * changes from firmware to software and clear the animation slot.
     */
    private static final class HardwareAnimPattern implements CANdlePattern {
        private final CANdlePattern impl;

        HardwareAnimPattern(CANdlePattern impl) {
            this.impl = impl;
        }

        @Override
        public void applyTo(CANdle candle, int startIdx, int numLeds) {
            impl.applyTo(candle, startIdx, numLeds);
        }
    }

    private static CANdlePattern hardwareAnim(CANdlePattern p) {
        return new HardwareAnimPattern(p);
    }

    /**
     * Device and LED range settings. The CAN device constructor owns and configures a new CANdle.
     * The shared device constructor applies no hardware config and targets a segment of a CANdle
     * another instance owns.
     */
    public static class Config {
        @Getter private String name;

        @Getter @Setter private boolean attached = true;

        /** When set, this instance reuses that CANdle and ignores deviceId and canBus. */
        @Getter @Setter private CANdle sharedCandle = null;

        @Getter @Setter private int deviceId = 1;

        @Getter @Setter private CANBus canBus;

        /** 0 includes the CANdle's 8 onboard status LEDs, so an external strip starts at 8. */
        @Getter @Setter private int startIdx = 0;

        @Getter @Setter private int numLeds;

        /**
         * Firmware animation slot, 0 to 7. Instances that share a CANdle need distinct slots or
         * their animations overwrite each other.
         */
        @Getter @Setter private int animationSlot = 0;

        @Getter @Setter private StripTypeValue stripType = StripTypeValue.RGB;

        /** Hardware brightness scalar, 0.0 to 1.0. */
        @Getter @Setter private double brightness = 1.0;

        @Getter @Setter
        private LossOfSignalBehaviorValue lossOfSignalBehavior =
                LossOfSignalBehaviorValue.DisableLEDs;

        /**
         * Configures a standalone strip that owns its own CANdle, applying the strip type,
         * brightness, and loss-of-signal behavior here.
         *
         * @param numLeds LEDs on the external strip, not counting the CANdle's 8 onboard LEDs
         */
        public Config(String name, int deviceId, int numLeds, CANBus canBus) {
            this.name = name;
            this.deviceId = deviceId;
            this.numLeds = numLeds;
            this.canBus = canBus;
            this.startIdx = 8;
        }

        /** Configures a segment of a CANdle another instance already configured. */
        public Config(String name, CANdle sharedCandle, int startIdx, int numLeds) {
            this.name = name;
            this.sharedCandle = sharedCandle;
            this.startIdx = startIdx;
            this.numLeds = numLeds;
        }
    }

    @Getter private Config config;

    @Getter protected final CANdle candle;

    private boolean lastWasAnimation = false;

    /** Orange blink, shown whenever no other command owns this subsystem. */
    protected final CANdlePattern defaultPattern;

    /**
     * Installed with {@code setDefaultCommand} here, so a subclass has to install its own to
     * replace it. Query whichever one is live through {@link #getDefaultCommand()}.
     */
    protected final Command defaultCommand;

    /** Active while the installed default command runs, so no other pattern owns the subsystem. */
    public final Trigger defaultTrigger;

    /** Priority of the running pattern command, or -1 when none is running. */
    @Getter @Setter private int commandPriority = -1;

    /** Spectrum purple, RGB 130, 103, 185. */
    public final Color purple = new Color(130, 103, 185);

    public final Color white = Color.kWhite;

    public SpectrumLEDs(Config config) {
        this.config = config;

        if (config.getSharedCandle() != null) {
            candle = config.getSharedCandle();
        } else {
            candle = new CANdle(config.getDeviceId(), config.getCanBus());
            CANdleConfiguration candleConfig =
                    new CANdleConfiguration()
                            .withLED(
                                    new LEDConfigs()
                                            .withStripType(config.getStripType())
                                            .withBrightnessScalar(config.getBrightness())
                                            .withLossOfSignalBehavior(
                                                    config.getLossOfSignalBehavior()));
            candle.getConfigurator().apply(candleConfig);
        }

        defaultPattern = blink(Color.kOrange, 1.0);

        defaultCommand = setPattern(defaultPattern, -1).withName("LEDs.defaultCommand");
        setDefaultCommand(defaultCommand);
        defaultTrigger =
                new Trigger(
                        () -> {
                            Command current = getCurrentCommand();
                            return current != null && current == getDefaultCommand();
                        });

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public boolean isAttached() {
        return config.isAttached();
    }

    public boolean isAnimating() {
        return lastWasAnimation;
    }

    /** Command name for telemetry, or "None" when nothing is running. */
    public String getCurrentCommandName() {
        Command cmd = getCurrentCommand();
        return cmd != null ? cmd.getName() : "None";
    }

    /**
     * Active while the running pattern's priority is at or below {@code priority}, so a caller can
     * yield to a higher-priority pattern.
     */
    public Trigger checkPriority(int priority) {
        return new Trigger(() -> commandPriority <= priority);
    }

    /**
     * Applies {@code pattern} every loop and holds {@code priority} in {@link #commandPriority}
     * until the command ends, then resets it to -1. The command runs even while the robot is
     * disabled.
     */
    public Command setPattern(CANdlePattern pattern, int priority) {
        return run(() -> {
                    commandPriority = priority;
                    boolean isAnim = pattern instanceof HardwareAnimPattern;
                    // Only our own slot is cleared, so instances sharing the CANdle keep theirs.
                    if (lastWasAnimation && !isAnim) {
                        candle.setControl(new EmptyAnimation(config.getAnimationSlot()));
                    }
                    lastWasAnimation = isAnim;
                    pattern.applyTo(candle, config.getStartIdx(), config.getNumLeds());
                })
                .finallyDo(() -> commandPriority = -1)
                .ignoringDisable(true)
                .withName("LEDs.setPattern");
    }

    public Command setPattern(CANdlePattern pattern) {
        return setPattern(pattern, 0);
    }

    /** Scales a {@link Color}'s 0 to 1 components to 0 to 255 and leaves W at 0. */
    private static RGBWColor toRGBW(Color color) {
        return new RGBWColor(
                (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 0);
    }

    /** Strobes {@code color} and off, each half-cycle lasting {@code onTimeSecs}. */
    public CANdlePattern blink(Color color, double onTimeSecs) {
        RGBWColor rgbw = toRGBW(color);
        // Built on first applyTo, once startIdx and numLeds are known.
        StrobeAnimation[] holder = new StrobeAnimation[1];
        return hardwareAnim(
                (candle, startIdx, numLeds) -> {
                    if (holder[0] == null) {
                        holder[0] =
                                new StrobeAnimation(startIdx, startIdx + numLeds - 1)
                                        .withSlot(config.getAnimationSlot())
                                        .withColor(rgbw)
                                        .withFrameRate(Hertz.of(1.0 / onTimeSecs));
                    }
                    candle.setControl(holder[0]);
                });
    }

    /** Fades between {@code color} and off once per {@code periodSecs}. */
    public CANdlePattern breathe(Color color, double periodSecs) {
        RGBWColor rgbw = toRGBW(color);
        SingleFadeAnimation[] holder = new SingleFadeAnimation[1];
        return hardwareAnim(
                (candle, startIdx, numLeds) -> {
                    if (holder[0] == null) {
                        holder[0] =
                                new SingleFadeAnimation(startIdx, startIdx + numLeds - 1)
                                        .withSlot(config.getAnimationSlot())
                                        .withColor(rgbw)
                                        .withFrameRate(Hertz.of(200.0 / periodSecs));
                    }
                    candle.setControl(holder[0]);
                });
    }

    public CANdlePattern rainbow() {
        return rainbow(1.0);
    }

    /**
     * Rainbow, dimmed by a hardware brightness scalar.
     *
     * @param brightness 0.0 to 1.0
     */
    public CANdlePattern rainbow(double brightness) {
        RainbowAnimation[] holder = new RainbowAnimation[1];
        return hardwareAnim(
                (candle, startIdx, numLeds) -> {
                    if (holder[0] == null) {
                        holder[0] =
                                new RainbowAnimation(startIdx, startIdx + numLeds - 1)
                                        .withSlot(config.getAnimationSlot())
                                        .withBrightness(brightness)
                                        .withFrameRate(Hertz.of(3));
                    }
                    candle.setControl(holder[0]);
                });
    }

    public CANdlePattern scrollingRainbow() {
        RainbowAnimation[] holder = new RainbowAnimation[1];
        return hardwareAnim(
                (candle, startIdx, numLeds) -> {
                    if (holder[0] == null) {
                        holder[0] =
                                new RainbowAnimation(startIdx, startIdx + numLeds - 1)
                                        .withSlot(config.getAnimationSlot())
                                        .withBrightness(1.0)
                                        .withFrameRate(Hertz.of(60));
                    }
                    candle.setControl(holder[0]);
                });
    }

    /**
     * Lights one LED at a time along the segment and repeats.
     *
     * @param speed full passes along the segment per second
     */
    public CANdlePattern chase(Color color, double speed) {
        RGBWColor rgbw = toRGBW(color);
        ColorFlowAnimation[] holder = new ColorFlowAnimation[1];
        return hardwareAnim(
                (candle, startIdx, numLeds) -> {
                    if (holder[0] == null) {
                        holder[0] =
                                new ColorFlowAnimation(startIdx, startIdx + numLeds - 1)
                                        .withSlot(config.getAnimationSlot())
                                        .withColor(rgbw)
                                        .withFrameRate(Hertz.of(numLeds * speed));
                    }
                    candle.setControl(holder[0]);
                });
    }

    public CANdlePattern bounce(Color color, double durationSecs) {
        RGBWColor rgbw = toRGBW(color);
        LarsonAnimation[] holder = new LarsonAnimation[1];
        return hardwareAnim(
                (candle, startIdx, numLeds) -> {
                    if (holder[0] == null) {
                        // One full cycle = 2 * (numLeds - 1) LED-position advances.
                        double frameRate = 2.0 * Math.max(numLeds - 1, 1) / durationSecs;
                        holder[0] =
                                new LarsonAnimation(startIdx, startIdx + numLeds - 1)
                                        .withSlot(config.getAnimationSlot())
                                        .withColor(rgbw)
                                        .withSize(3)
                                        .withBounceMode(LarsonBounceValue.Back)
                                        .withFrameRate(Hertz.of(frameRate));
                    }
                    candle.setControl(holder[0]);
                });
    }

    public CANdlePattern fire() {
        FireAnimation[] holder = new FireAnimation[1];
        return hardwareAnim(
                (candle, startIdx, numLeds) -> {
                    if (holder[0] == null) {
                        holder[0] =
                                new FireAnimation(startIdx, startIdx + numLeds - 1)
                                        .withSlot(config.getAnimationSlot())
                                        .withFrameRate(Hertz.of(60));
                    }
                    candle.setControl(holder[0]);
                });
    }

    public CANdlePattern rgbCycle() {
        RgbFadeAnimation[] holder = new RgbFadeAnimation[1];
        return hardwareAnim(
                (candle, startIdx, numLeds) -> {
                    if (holder[0] == null) {
                        holder[0] =
                                new RgbFadeAnimation(startIdx, startIdx + numLeds - 1)
                                        .withSlot(config.getAnimationSlot())
                                        .withFrameRate(Hertz.of(30));
                    }
                    candle.setControl(holder[0]);
                });
    }

    public CANdlePattern solid(Color color) {
        RGBWColor rgbw = toRGBW(color);
        SolidColor[] holder = new SolidColor[1];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                holder[0] = new SolidColor(startIdx, startIdx + numLeds - 1).withColor(rgbw);
            }
            candle.setControl(holder[0]);
        };
    }

    /**
     * Splits the segment, showing {@code color1} on the leading LEDs and {@code color2} on the
     * rest.
     *
     * @param percent share of the segment for {@code color1}, 0.0 to 1.0
     */
    public CANdlePattern stripe(double percent, Color color1, Color color2) {
        RGBWColor rgbw1 = toRGBW(color1);
        RGBWColor rgbw2 = toRGBW(color2);
        SolidColor[][] holder = new SolidColor[1][];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                int split = Math.max(0, Math.min((int) Math.round(numLeds * percent), numLeds));
                holder[0] = new SolidColor[2];
                holder[0][0] =
                        (split > 0)
                                ? new SolidColor(startIdx, startIdx + split - 1).withColor(rgbw1)
                                : null;
                holder[0][1] =
                        (split < numLeds)
                                ? new SolidColor(startIdx + split, startIdx + numLeds - 1)
                                        .withColor(rgbw2)
                                : null;
            }
            for (SolidColor req : holder[0]) {
                if (req != null) candle.setControl(req);
            }
        };
    }

    public CANdlePattern gradient(Color color1, Color color2) {
        SolidColor[][] holder = new SolidColor[1][];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                holder[0] = new SolidColor[numLeds];
                for (int i = 0; i < numLeds; i++) {
                    double ratio = (numLeds <= 1) ? 0.0 : (double) i / (numLeds - 1);
                    int r = (int) (color1.red * 255 * (1 - ratio) + color2.red * 255 * ratio);
                    int g = (int) (color1.green * 255 * (1 - ratio) + color2.green * 255 * ratio);
                    int b = (int) (color1.blue * 255 * (1 - ratio) + color2.blue * 255 * ratio);
                    holder[0][i] =
                            new SolidColor(startIdx + i, startIdx + i)
                                    .withColor(new RGBWColor(r, g, b, 0));
                }
            }
            for (SolidColor req : holder[0]) {
                candle.setControl(req);
            }
        };
    }

    /**
     * Lights {@code length} LEDs at each end and blanks the middle, clamping {@code length} to half
     * the segment.
     */
    public CANdlePattern edges(Color color, int length) {
        RGBWColor rgbw = toRGBW(color);
        SolidColor[][] holder = new SolidColor[1][];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                int clampedLen = Math.min(length, numLeds / 2);
                if (clampedLen == 0) {
                    holder[0] = new SolidColor[0];
                } else {
                    int centerStart = startIdx + clampedLen;
                    int centerEnd = startIdx + numLeds - clampedLen - 1;
                    holder[0] = (centerStart <= centerEnd) ? new SolidColor[3] : new SolidColor[2];
                    holder[0][0] =
                            new SolidColor(startIdx, startIdx + clampedLen - 1).withColor(rgbw);
                    holder[0][1] =
                            new SolidColor(startIdx + numLeds - clampedLen, startIdx + numLeds - 1)
                                    .withColor(rgbw);
                    if (holder[0].length == 3) {
                        holder[0][2] =
                                new SolidColor(centerStart, centerEnd)
                                        .withColor(new RGBWColor(0, 0, 0, 0));
                    }
                }
            }
            for (SolidColor req : holder[0]) {
                candle.setControl(req);
            }
        };
    }

    public CANdlePattern ombre(Color startColor, Color endColor) {
        SolidColor[][] holder = new SolidColor[1][];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                holder[0] = new SolidColor[numLeds];
                for (int i = 0; i < numLeds; i++) {
                    holder[0][i] = new SolidColor(startIdx + i, startIdx + i);
                }
            }
            // Blend scrolls 0.58 strip lengths per second.
            double phaseShift = (System.currentTimeMillis() / 1000.0) * 0.58 % 1.0;
            for (int i = 0; i < numLeds; i++) {
                double ratio = ((i + numLeds * phaseShift) / numLeds) % 1.0;
                int r = (int) (startColor.red * 255 * (1 - ratio) + endColor.red * 255 * ratio);
                int g = (int) (startColor.green * 255 * (1 - ratio) + endColor.green * 255 * ratio);
                int b = (int) (startColor.blue * 255 * (1 - ratio) + endColor.blue * 255 * ratio);
                holder[0][i].Color = new RGBWColor(r, g, b, 0);
                candle.setControl(holder[0][i]);
            }
        };
    }

    /**
     * Sine wave between {@code c1} and {@code c2} along the segment.
     *
     * @param cycleLength LEDs per wave period
     */
    public CANdlePattern wave(Color c1, Color c2, double cycleLength, double durationSecs) {
        SolidColor[][] holder = new SolidColor[1][];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                holder[0] = new SolidColor[numLeds];
                for (int i = 0; i < numLeds; i++) {
                    holder[0][i] = new SolidColor(startIdx + i, startIdx + i);
                }
            }
            double currentTime = Timer.getFPGATimestamp();
            double phase = (currentTime % durationSecs) / durationSecs;
            double x = (1 - phase) * 2.0 * Math.PI;
            double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
            double waveExponent = 0.4;
            for (int i = 0; i < numLeds; i++) {
                x += xDiffPerLed;
                double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
                if (Double.isNaN(ratio)) {
                    ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
                }
                if (Double.isNaN(ratio)) ratio = 0.5;
                int r = (int) (c1.red * 255 * (1 - ratio) + c2.red * 255 * ratio);
                int g = (int) (c1.green * 255 * (1 - ratio) + c2.green * 255 * ratio);
                int b = (int) (c1.blue * 255 * (1 - ratio) + c2.blue * 255 * ratio);
                holder[0][i].Color = new RGBWColor(r, g, b, 0);
                candle.setControl(holder[0][i]);
            }
        };
    }

    /**
     * Blanks LEDs from the end of the segment toward the front as the time runs out.
     *
     * @param countStartTimeSec FPGA timestamp, in seconds, when the countdown began
     * @param durationInSeconds length of the countdown
     */
    public CANdlePattern countdown(DoubleSupplier countStartTimeSec, double durationInSeconds) {
        SolidColor[][] holder = new SolidColor[1][];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                holder[0] = new SolidColor[numLeds];
                for (int i = 0; i < numLeds; i++) {
                    holder[0][i] = new SolidColor(startIdx + i, startIdx + i);
                }
            }
            // Read the supplier each loop, so a pattern built at binding time still measures from
            // the start time in force when the command runs.
            double elapsed = Timer.getFPGATimestamp() - countStartTimeSec.getAsDouble();
            double progress = Math.min(elapsed / durationInSeconds, 1.0);
            int ledsOff = (int) (numLeds * progress);
            int green = (int) (255 * (1 - progress));
            for (int i = numLeds - 1; i >= 0; i--) {
                holder[0][i].Color =
                        (numLeds - i <= ledsOff)
                                ? new RGBWColor(0, 0, 0, 0)
                                : new RGBWColor(255, green, 0, 0);
                candle.setControl(holder[0][i]);
            }
        };
    }

    /**
     * Switch countdown. Match time remaining picks the color and blanks the segment, on this
     * schedule:
     *
     * <pre>
     *  140 to 130  purple
     *  130 to 105  startingColor
     *  105 to 80   opponent color
     *   80 to 55   startingColor
     *   55 to 30   opponent color
     *   30 to 0    purple
     * </pre>
     */
    public CANdlePattern switchCountdown(Color startingColor) {
        SolidColor[][] holder = new SolidColor[1][];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                holder[0] = new SolidColor[numLeds];
                for (int i = 0; i < numLeds; i++) {
                    holder[0][i] = new SolidColor(startIdx + i, startIdx + i);
                }
            }

            int[] times = {10, 25, 25, 25, 25, 30};
            double elapsed = 140 - Timer.getMatchTime();

            int shiftTime = 0;
            int cumulativeTime = 0;
            Color color = Color.kBlack;

            for (int i = 0; i < times.length; i++) {
                cumulativeTime += times[i];
                if (cumulativeTime > elapsed) {
                    shiftTime = times[i];
                    switch (i) {
                        case 0, 5 -> color = Color.kPurple;
                        case 1, 3 -> color = startingColor;
                        case 2, 4 -> color =
                                Color.kRed.equals(startingColor) ? Color.kBlue : Color.kRed;
                        default -> color = Color.kBlack;
                    }
                    break;
                }
            }

            double progress = 1.0 - (cumulativeTime - elapsed) / Math.max(shiftTime, 1);
            int ledsOff = (int) (numLeds * Math.min(progress, 1.0));
            RGBWColor segColor = toRGBW(color);

            for (int i = numLeds - 1; i >= 0; i--) {
                holder[0][i].Color =
                        (numLeds - i <= ledsOff) ? new RGBWColor(0, 0, 0, 0) : segColor;
                candle.setControl(holder[0][i]);
            }
        };
    }
}
