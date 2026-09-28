package frc.spectrumLib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;

/** Math and driver station helpers. Based on 254's Util, imported from 1678-2024. */
public class Util {

    /** Tolerance {@link #epsilonEquals(double, double)} uses for floating point equality. */
    public static final double EPSILON = 1e-12;

    /** Prevents instantiation of this utility class. */
    private Util() {}

    /** Clamps {@code v} to [-{@code maxMagnitude}, {@code maxMagnitude}]. */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    /** Clamps {@code v} to [{@code min}, {@code max}], both bounds inclusive. */
    public static double limit(double v, double min, double max) {
        return MathUtil.clamp(v, min, max);
    }

    /** True when the magnitude of {@code v} is under {@code maxMagnitude}, bounds exclusive. */
    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    /** True when {@code v} sits between {@code min} and {@code max}, both bounds exclusive. */
    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    /**
     * Reads each supplier once and tests the result against the two bounds. Use this instead of
     * calling {@link #inRange(double, double, double)} on values that are still moving.
     */
    public static boolean inRange(DoubleSupplier v, DoubleSupplier min, DoubleSupplier max) {
        double value = v.getAsDouble();
        return value > min.getAsDouble() && value < max.getAsDouble();
    }

    /** Interpolates from {@code a} at {@code x = 0} to {@code b} at {@code x = 1}, clamping x. */
    public static double interpolate(double a, double b, double x) {
        return MathUtil.interpolate(a, b, x);
    }

    public static String joinStrings(final String delim, final List<?> strings) {
        return strings.stream().map(Object::toString).collect(Collectors.joining(delim));
    }

    /** True when {@code a} and {@code b} differ by {@code epsilon} or less. */
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    /**
     * Widens to long before subtracting, so comparing values near the int limits cannot overflow
     * into a wrong answer.
     */
    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return ((long) a - epsilon <= b) && ((long) a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        return list.stream().allMatch(v -> epsilonEquals(v, value, epsilon));
    }

    public static final Trigger teleop = RobotModeTriggers.teleop();

    public static final Trigger autoMode = RobotModeTriggers.autonomous();

    public static final Trigger testMode = RobotModeTriggers.test();

    public static final Trigger disabled = RobotModeTriggers.disabled();

    public static final Trigger dsAttached = new Trigger(DriverStation::isDSAttached);
}
