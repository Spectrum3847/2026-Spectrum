package frc.spectrumLib.leds;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
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
import frc.spectrumLib.hardware.CanConfigBudget;
import java.util.function.DoubleSupplier;
import java.util.function.IntFunction;
import lombok.Getter;
import lombok.Setter;

/**
 * CANdle-based addressable LED subsystem, with pattern factories for solids, stripes, blinks,
 * breathes, rainbows, chases, bounces, gradients, ombres, waves, and countdowns.
 *
 * <p>Patterns come in two flavors. Hardware animations ({@link #blink}, {@link #breathe}, {@link
 * #rainbow}, {@link #scrollingRainbow}, {@link #chase}, {@link #bounce}, {@link #fire}, {@link
 * #rgbCycle}) run in the CANdle's firmware and cost nothing per loop. Software patterns ({@link
 * #solid}, {@link #stripe}, {@link #gradient}, {@link #ombre}, {@link #wave}, {@link #countdown},
 * {@link #switchCountdown}, {@link #edges}) send a one-shot {@link SolidColor} that is resent every
 * loop, and switching to one of them clears the running animations.
 *
 * <p>Several instances can share one physical {@link CANdle} by passing the same device to their
 * {@link Config} and taking non-overlapping {@code startIdx} and {@code numLeds} ranges, each with
 * its own animation slot.
 *
 * <p>{@link #setPattern(CANdlePattern, int)} returns a {@link Command} that applies the pattern for
 * as long as it runs and holds the priority it was given ({@link #checkPriority(int)}).
 */
public class SpectrumLEDs implements Subsystem {

    /**
     * Functional interface for a LED pattern that drives a segment of a {@link CANdle} strip.
     *
     * <p>Implementations receive the live {@link CANdle} device, the first LED index in this
     * instance's segment ({@code startIdx}), and the number of LEDs in the segment ({@code
     * numLeds}). Hardware animation patterns call {@link CANdle#setControl}, software patterns call
     * {@link CANdle#setControl(SolidColor)} (one-shot, resent each loop).
     */
    @FunctionalInterface
    public interface CANdlePattern {
        /**
         * Apply this pattern to the given LED segment.
         *
         * @param candle the {@link CANdle} device to write to
         * @param startIdx the first LED index (inclusive); {@code 0} includes the 8 onboard LEDs
         * @param numLeds the number of LEDs in this segment
         */
        void applyTo(CANdle candle, int startIdx, int numLeds);
    }

    /**
     * Marker wrapper around a hardware animation, so {@link #setPattern(CANdlePattern, int)} can
     * see that the pattern has to clear the animation slots when it is swapped out.
     */
    private record HardwareAnimPattern(CANdlePattern impl) implements CANdlePattern {
        @Override
        public void applyTo(CANdle candle, int startIdx, int numLeds) {
            impl.applyTo(candle, startIdx, numLeds);
        }
    }

    @FunctionalInterface
    private interface SegmentRequest {
        ControlRequest build(int startIdx, int numLeds);
    }

    /**
     * A pattern that sends one request, built on the first call from the runtime segment and resent
     * every call after that.
     */
    private static CANdlePattern lazy(SegmentRequest request) {
        ControlRequest[] holder = new ControlRequest[1];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                holder[0] = request.build(startIdx, numLeds);
            }
            candle.setControl(holder[0]);
        };
    }

    /** {@link #lazy} marked as a hardware animation. */
    private static CANdlePattern lazyAnim(SegmentRequest request) {
        return new HardwareAnimPattern(lazy(request));
    }

    /** Gives each LED's color for one loop; built fresh every loop from the segment length. */
    @FunctionalInterface
    private interface LedFrame {
        IntFunction<RGBWColor> colors(int numLeds);
    }

    /** A software pattern that writes every LED individually each loop. */
    private static CANdlePattern perLed(LedFrame frame) {
        SolidColor[][] holder = new SolidColor[1][];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                holder[0] = new SolidColor[numLeds];
                for (int i = 0; i < numLeds; i++) {
                    holder[0][i] = new SolidColor(startIdx + i, startIdx + i);
                }
            }
            IntFunction<RGBWColor> colorAt = frame.colors(numLeds);
            for (int i = 0; i < numLeds; i++) {
                holder[0][i].Color = colorAt.apply(i);
                candle.setControl(holder[0][i]);
            }
        };
    }

    /**
     * Configuration for a {@link SpectrumLEDs} subsystem instance.
     *
     * <p>Use {@link #Config(String, int, int, CANBus)} for an instance that owns and configures its
     * own {@link CANdle}, or {@link #Config(String, CANdle, int, int)} to share a device that is
     * already configured.
     */
    public static class Config {
        /** Human-readable name used in telemetry. */
        @Getter private String name;

        /** Whether this LED strip is physically connected to the robot. */
        @Getter @Setter private boolean attached = true;

        /**
         * Pre-built {@link CANdle} to reuse. When non-null, {@link #deviceId} and {@link #canBus}
         * are ignored and this instance applies no hardware configuration.
         */
        @Getter @Setter private CANdle sharedCandle = null;

        /** CAN device ID used when no {@link #sharedCandle} is provided. */
        @Getter @Setter private int deviceId = 1;

        /** CAN bus used when no {@link #sharedCandle} is provided. */
        @Getter @Setter private CANBus canBus;

        /**
         * First LED index (inclusive) in the strip. Use {@code 0} to include the 8 onboard status
         * LEDs on the CANdle board itself; use {@code 8} to skip them.
         */
        @Getter @Setter private int startIdx = 0;

        /** Number of LEDs in the segment owned by this instance. */
        @Getter @Setter private int numLeds;

        /**
         * CANdle hardware animation slot (0-7) used by this instance's animation patterns. Each
         * {@link SpectrumLEDs} instance sharing a single {@link CANdle} must use a distinct slot,
         * otherwise their animations overwrite each other.
         */
        @Getter @Setter private int animationSlot = 0;

        /** LED strip type (RGB, RGBW, GRB, etc.). Ignored when {@link #sharedCandle} is set. */
        @Getter @Setter private StripTypeValue stripType = StripTypeValue.RGB;

        /**
         * Overall brightness scalar applied in hardware (0.0-1.0). Ignored when {@link
         * #sharedCandle} is set.
         */
        @Getter @Setter private double brightness = 1.0;

        /**
         * Behavior of the strip when CAN signal is lost. Ignored when {@link #sharedCandle} is set.
         */
        @Getter @Setter
        private LossOfSignalBehaviorValue lossOfSignalBehavior =
                LossOfSignalBehaviorValue.DisableLEDs;

        /**
         * Configures an instance that creates and owns its own {@link CANdle}, applying strip type,
         * brightness, and loss-of-signal behavior to the hardware. {@code startIdx} starts at 8 so
         * the onboard LEDs are skipped.
         *
         * @param name human-readable name for telemetry
         * @param deviceId CAN device ID of the CANdle
         * @param numLeds number of LEDs on the external strip, not counting the 8 onboard LEDs
         * @param canBus CAN bus the CANdle is on
         */
        public Config(String name, int deviceId, int numLeds, CANBus canBus) {
            this.name = name;
            this.deviceId = deviceId;
            this.numLeds = numLeds;
            this.canBus = canBus;
            this.startIdx = 8;
        }

        /**
         * Configures a zone that shares an already-configured {@link CANdle}. No hardware
         * configuration is applied.
         *
         * @param name human-readable name for telemetry
         * @param sharedCandle the {@link CANdle} to reuse
         * @param startIdx first LED index (inclusive) in the shared strip for this zone
         * @param numLeds number of LEDs in this zone
         */
        public Config(String name, CANdle sharedCandle, int startIdx, int numLeds) {
            this.name = name;
            this.sharedCandle = sharedCandle;
            this.startIdx = startIdx;
            this.numLeds = numLeds;
        }
    }

    @Getter private Config config;

    /** The {@link CANdle} this instance drives, either owned or shared. */
    @Getter protected final CANdle candle;

    private boolean lastWasAnimation = false;

    /**
     * Pattern shown when no other command wants this subsystem (an orange blink). Built in the
     * constructor body, after {@link #config} is set, so the factory can read the config.
     */
    protected final CANdlePattern defaultPattern;

    /**
     * The default command, which displays {@link #defaultPattern} at the lowest priority. A
     * subclass can install its own default command to replace it; query the installed one with
     * {@link Subsystem#getDefaultCommand()}.
     */
    protected final Command defaultCommand;

    /**
     * Active while the installed default command is the one running, which means no higher-priority
     * pattern owns the subsystem.
     */
    public final Trigger defaultTrigger;

    /**
     * Priority of the pattern command currently running; {@link #setPattern(CANdlePattern, int)}
     * stores this while a command runs and resets it to {@code -1} when the command ends.
     */
    @Getter @Setter private int commandPriority = -1;

    /** Spectrum purple color constant ({@code RGB 130, 103, 185}). */
    public final Color purple = new Color(130, 103, 185);

    /** Convenience alias for {@link Color#kWhite}. */
    public final Color white = Color.kWhite;

    /**
     * Configures the hardware (or reuses a shared device) and registers with the WPILib {@link
     * CommandScheduler}.
     *
     * @param config device, segment range, and strip type for this instance
     */
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
            CanConfigBudget.run(
                    "CANdle " + config.getDeviceId(),
                    timeout -> candle.getConfigurator().apply(candleConfig, timeout));
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

    /**
     * True when the most recently applied pattern was a hardware animation running in firmware,
     * false when it was a software {@link SolidColor} pattern.
     */
    public boolean isAnimating() {
        return lastWasAnimation;
    }

    /** Name of the command currently applying a pattern, or "None" when none is running. */
    public String getCurrentCommandName() {
        Command cmd = getCurrentCommand();
        return cmd != null ? cmd.getName() : "None";
    }

    /**
     * Returns a trigger that is true while the running pattern command's priority is at or below
     * {@code priority}, for gating a lower-priority pattern out of the way of a higher-priority
     * one.
     */
    public Trigger checkPriority(int priority) {
        return new Trigger(() -> commandPriority <= priority);
    }

    /**
     * Returns a command that applies {@code pattern} to the segment for as long as it runs and
     * holds {@code priority} while it does. The command runs while the robot is disabled.
     *
     * <p>Switching from a hardware animation to a software {@link SolidColor} pattern clears the
     * active animation slots before the first software write.
     *
     * @param pattern pattern to apply each loop cycle
     * @param priority priority level to hold in {@link #commandPriority} while this command runs
     * @return a command that applies the pattern continuously
     */
    public Command setPattern(CANdlePattern pattern, int priority) {
        return run(() -> {
                    commandPriority = priority;
                    boolean isAnim = pattern instanceof HardwareAnimPattern;
                    // Clear this instance's animation slot once when transitioning to a software
                    // pattern. Only our own slot is cleared so other instances sharing the same
                    // CANdle keep their animations running.
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

    /** {@link #setPattern(CANdlePattern, int)} at priority 0. */
    public Command setPattern(CANdlePattern pattern) {
        return setPattern(pattern, 0);
    }

    /**
     * Converts a WPILib {@link Color} (0.0-1.0 double components) to a {@link RGBWColor} with
     * {@code W = 0}.
     */
    private static RGBWColor toRGBW(Color color) {
        return new RGBWColor(
                (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 0);
    }

    /** Blends {@code a} toward {@code b} by {@code ratio} (0 = a, 1 = b). */
    private static RGBWColor lerp(Color a, Color b, double ratio) {
        return new RGBWColor(
                (int) (a.red * 255 * (1 - ratio) + b.red * 255 * ratio),
                (int) (a.green * 255 * (1 - ratio) + b.green * 255 * ratio),
                (int) (a.blue * 255 * (1 - ratio) + b.blue * 255 * ratio),
                0);
    }

    private static final RGBWColor OFF = new RGBWColor(0, 0, 0, 0);

    /**
     * Blinks between {@code color} and off. Each half-cycle, on and off, lasts {@code onTimeSecs}
     * seconds, so the frame rate is one over that.
     *
     * @param color the blink color
     * @param onTimeSecs seconds per on or off half-cycle
     * @return a hardware animation {@link CANdlePattern}
     */
    public CANdlePattern blink(Color color, double onTimeSecs) {
        RGBWColor rgbw = toRGBW(color);
        return lazyAnim(
                (startIdx, numLeds) ->
                        new StrobeAnimation(startIdx, startIdx + numLeds - 1)
                                .withSlot(config.getAnimationSlot())
                                .withColor(rgbw)
                                .withFrameRate(Hertz.of(1.0 / onTimeSecs)));
    }

    /**
     * Fades between {@code color} and off and back, over {@code periodSecs}. Each animation frame
     * moves brightness by 1%, so a full cycle needs 200 frames and the frame rate is 200 over the
     * period.
     *
     * @param color the peak color at full brightness
     * @param periodSecs seconds for one full breathe cycle
     * @return a hardware animation {@link CANdlePattern}
     */
    public CANdlePattern breathe(Color color, double periodSecs) {
        RGBWColor rgbw = toRGBW(color);
        return lazyAnim(
                (startIdx, numLeds) ->
                        new SingleFadeAnimation(startIdx, startIdx + numLeds - 1)
                                .withSlot(config.getAnimationSlot())
                                .withColor(rgbw)
                                .withFrameRate(Hertz.of(200.0 / periodSecs)));
    }

    /** Rainbow advancing slowly across the strip. */
    public CANdlePattern rainbow() {
        return rainbow(1.0);
    }

    /**
     * Rainbow advancing slowly across the strip, at the given brightness.
     *
     * @param brightness brightness scalar, 0.0-1.0
     */
    public CANdlePattern rainbow(double brightness) {
        return lazyAnim(
                (startIdx, numLeds) ->
                        new RainbowAnimation(startIdx, startIdx + numLeds - 1)
                                .withSlot(config.getAnimationSlot())
                                .withBrightness(brightness)
                                .withFrameRate(Hertz.of(3)));
    }

    /** Rainbow advancing quickly across the strip. */
    public CANdlePattern scrollingRainbow() {
        return lazyAnim(
                (startIdx, numLeds) ->
                        new RainbowAnimation(startIdx, startIdx + numLeds - 1)
                                .withSlot(config.getAnimationSlot())
                                .withBrightness(1.0)
                                .withFrameRate(Hertz.of(60)));
    }

    /**
     * Lights one LED at a time across the strip and repeats. The frame rate is {@code numLeds *
     * speed}, so {@code speed} full cycles happen per second.
     *
     * @param color the chase color
     * @param speed desired full strip cycles per second
     * @return a hardware animation {@link CANdlePattern}
     */
    public CANdlePattern chase(Color color, double speed) {
        RGBWColor rgbw = toRGBW(color);
        return lazyAnim(
                (startIdx, numLeds) ->
                        new ColorFlowAnimation(startIdx, startIdx + numLeds - 1)
                                .withSlot(config.getAnimationSlot())
                                .withColor(rgbw)
                                .withFrameRate(Hertz.of(numLeds * speed)));
    }

    /**
     * Bouncing dot that travels back and forth along the strip.
     *
     * @param color the dot color
     * @param durationSecs seconds for one complete back-and-forth cycle
     * @return a hardware animation {@link CANdlePattern}
     */
    public CANdlePattern bounce(Color color, double durationSecs) {
        RGBWColor rgbw = toRGBW(color);
        return lazyAnim(
                (startIdx, numLeds) ->
                        new LarsonAnimation(startIdx, startIdx + numLeds - 1)
                                .withSlot(config.getAnimationSlot())
                                .withColor(rgbw)
                                .withSize(3)
                                .withBounceMode(LarsonBounceValue.Back)
                                // One full cycle = 2 * (numLeds - 1) LED-position advances.
                                .withFrameRate(
                                        Hertz.of(2.0 * Math.max(numLeds - 1, 1) / durationSecs)));
    }

    /** Fire animation from the CANdle's own animation engine. */
    public CANdlePattern fire() {
        return lazyAnim(
                (startIdx, numLeds) ->
                        new FireAnimation(startIdx, startIdx + numLeds - 1)
                                .withSlot(config.getAnimationSlot())
                                .withFrameRate(Hertz.of(60)));
    }

    /** RGB color cycle from the CANdle's own animation engine. */
    public CANdlePattern rgbCycle() {
        return lazyAnim(
                (startIdx, numLeds) ->
                        new RgbFadeAnimation(startIdx, startIdx + numLeds - 1)
                                .withSlot(config.getAnimationSlot())
                                .withFrameRate(Hertz.of(30)));
    }

    /**
     * Solid color, from a single {@link SolidColor} control request resent each loop.
     *
     * @param color the color to display
     * @return a software {@link CANdlePattern} showing a constant solid color
     */
    public CANdlePattern solid(Color color) {
        RGBWColor rgbw = toRGBW(color);
        return lazy(
                (startIdx, numLeds) ->
                        new SolidColor(startIdx, startIdx + numLeds - 1).withColor(rgbw));
    }

    /**
     * Two-color stripe: the first {@code percent} fraction of LEDs shows {@code color1} and the
     * rest shows {@code color2}, sent as two {@link SolidColor} controls.
     *
     * @param percent fraction of the strip (0.0-1.0) assigned to {@code color1}
     * @param color1 color for the leading segment
     * @param color2 color for the trailing segment
     * @return a software {@link CANdlePattern} showing the two-color stripe
     */
    public CANdlePattern stripe(double percent, Color color1, Color color2) {
        RGBWColor rgbw1 = toRGBW(color1);
        RGBWColor rgbw2 = toRGBW(color2);
        SolidColor[][] holder = new SolidColor[1][];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                int split = Math.max(0, Math.min((int) Math.round(numLeds * percent), numLeds));
                holder[0] = new SolidColor[2];
                // Segment 1 (may be empty if split == 0)
                holder[0][0] =
                        (split > 0)
                                ? new SolidColor(startIdx, startIdx + split - 1).withColor(rgbw1)
                                : null;
                // Segment 2 (may be empty if split == numLeds)
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

    /**
     * Linear gradient from {@code color1} to {@code color2} across the segment, one {@link
     * SolidColor} per LED, computed at first use.
     *
     * @param color1 color at the start (index 0) of the segment
     * @param color2 color at the end of the segment
     * @return a software {@link CANdlePattern} showing the two-color gradient
     */
    public CANdlePattern gradient(Color color1, Color color2) {
        SolidColor[][] holder = new SolidColor[1][];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                holder[0] = new SolidColor[numLeds];
                for (int i = 0; i < numLeds; i++) {
                    double ratio = (numLeds <= 1) ? 0.0 : (double) i / (numLeds - 1);
                    holder[0][i] =
                            new SolidColor(startIdx + i, startIdx + i)
                                    .withColor(lerp(color1, color2, ratio));
                }
            }
            for (SolidColor req : holder[0]) {
                candle.setControl(req);
            }
        };
    }

    /**
     * Lights the first and last {@code length} LEDs with {@code color} and turns the middle off,
     * with two or three {@link SolidColor} controls.
     *
     * @param color the color to apply to the edge LEDs
     * @param length the number of LEDs to illuminate at each end of the strip
     * @return a software {@link CANdlePattern} showing lit edges and a dark center
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
                    // left edge, right edge, (optional) center black
                    holder[0] = (centerStart <= centerEnd) ? new SolidColor[3] : new SolidColor[2];
                    holder[0][0] =
                            new SolidColor(startIdx, startIdx + clampedLen - 1).withColor(rgbw);
                    holder[0][1] =
                            new SolidColor(startIdx + numLeds - clampedLen, startIdx + numLeds - 1)
                                    .withColor(rgbw);
                    if (holder[0].length == 3) {
                        holder[0][2] = new SolidColor(centerStart, centerEnd).withColor(OFF);
                    }
                }
            }
            for (SolidColor req : holder[0]) {
                candle.setControl(req);
            }
        };
    }

    /**
     * Ombre that blends between the two colors and scrolls the blend point along the strip over
     * time. The colors are rebuilt every loop because {@link RGBWColor} is immutable.
     *
     * @param startColor the leading color
     * @param endColor the trailing color
     * @return a software {@link CANdlePattern} showing the animated ombre
     */
    public CANdlePattern ombre(Color startColor, Color endColor) {
        return perLed(
                numLeds -> {
                    // Speed: 0.58 strip-lengths per second
                    double phaseShift = (System.currentTimeMillis() / 1000.0) * 0.58 % 1.0;
                    return i ->
                            lerp(
                                    startColor,
                                    endColor,
                                    ((i + numLeds * phaseShift) / numLeds) % 1.0);
                });
    }

    /**
     * Sine wave blending between the two colors, one {@link SolidColor} per LED.
     *
     * @param c1 first wave color
     * @param c2 second wave color
     * @param cycleLength number of LEDs per wave period
     * @param durationSecs period of the time-based animation in seconds
     * @return a software {@link CANdlePattern} showing the wave
     */
    public CANdlePattern wave(Color c1, Color c2, double cycleLength, double durationSecs) {
        double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
        double waveExponent = 0.4;
        return perLed(
                numLeds -> {
                    double phase = (Timer.getFPGATimestamp() % durationSecs) / durationSecs;
                    double x0 = (1 - phase) * 2.0 * Math.PI;
                    return i -> {
                        double x = x0 + (i + 1) * xDiffPerLed;
                        double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
                        if (Double.isNaN(ratio)) {
                            ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
                        }
                        if (Double.isNaN(ratio)) ratio = 0.5;
                        return lerp(c1, c2, ratio);
                    };
                });
    }

    /**
     * Counts down by turning LEDs off from the end of the segment toward the start, while the color
     * fades from yellow to red.
     *
     * @param countStartTimeSec supplies the FPGA timestamp (seconds) when the countdown began
     * @param durationInSeconds total countdown duration in seconds
     * @return a software {@link CANdlePattern} showing the countdown
     */
    public CANdlePattern countdown(DoubleSupplier countStartTimeSec, double durationInSeconds) {
        return perLed(
                numLeds -> {
                    // Read the supplier each loop (not at factory time) so patterns built at
                    // binding time still measure from the correct start when the command runs.
                    double elapsed = Timer.getFPGATimestamp() - countStartTimeSec.getAsDouble();
                    double progress = Math.min(elapsed / durationInSeconds, 1.0);
                    int ledsOff = (int) (numLeds * progress);
                    RGBWColor on = new RGBWColor(255, (int) (255 * (1 - progress)), 0, 0);
                    return i -> (numLeds - i <= ledsOff) ? OFF : on;
                });
    }

    /**
     * Alliance switch countdown. Colors follow a hard-coded match-time schedule, and LEDs turn off
     * within each segment as its time runs out.
     *
     * <p>Seconds remaining, and the color shown:
     *
     * <pre>
     *  140-130  purple
     *  130-105  startingColor
     *  105-80   opponent color
     *   80-55   startingColor
     *   55-30   opponent color
     *   30-0    purple
     * </pre>
     *
     * @param startingColor the alliance color shown during this robot's segments
     * @return a software {@link CANdlePattern} reflecting the current switch-countdown state
     */
    public CANdlePattern switchCountdown(Color startingColor) {
        int[] times = {10, 25, 25, 25, 25, 30};
        return perLed(
                numLeds -> {
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
                    return i -> (numLeds - i <= ledsOff) ? OFF : segColor;
                });
    }
}
