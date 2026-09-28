package frc.spectrumLib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;

/* From 254 lib, imported from 1678-2024. */
public class Util {

    /** Tolerance for the epsilonEquals checks. */
    public static final double EPSILON = 1e-12;

    /** Prevent this class from being instantiated. */
    private Util() {}

    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    /** Clamps v into [min, max], counting both bounds as inside the range. */
    public static double limit(double v, double min, double max) {
        return MathUtil.clamp(v, min, max);
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    /** True when v is strictly between min and max, so both bounds sit outside the range. */
    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    /**
     * Reads every supplier on each call, so a bound supplied as a live value can move between
     * checks.
     */
    public static boolean inRange(DoubleSupplier v, DoubleSupplier min, DoubleSupplier max) {
        double value = v.getAsDouble();
        return value > min.getAsDouble() && value < max.getAsDouble();
    }

    /**
     * Interpolates from a at x = 0 to b at x = 1. MathUtil clamps x to [0, 1], so a factor outside
     * that range does not extrapolate.
     */
    public static double interpolate(double a, double b, double x) {
        return MathUtil.interpolate(a, b, x);
    }

    /** Joins the elements' toString() values, separated by delim. */
    public static String joinStrings(final String delim, final List<?> strings) {
        return strings.stream().map(Object::toString).collect(Collectors.joining(delim));
    }

    /** True when |a - b| <= epsilon. */
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    /** True when |a - b| <= {@link #EPSILON}. */
    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    /** True when |a - b| <= epsilon, widened to long so a large epsilon cannot overflow. */
    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return ((long) a - epsilon <= b) && ((long) a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        return list.stream().allMatch(v -> epsilonEquals(v, value, epsilon));
    }

    /** True only while the robot is enabled in teleop. */
    public static final Trigger teleop = RobotModeTriggers.teleop();

    /** True only while the robot is enabled in autonomous. */
    public static final Trigger autoMode = RobotModeTriggers.autonomous();

    /** True only while the robot is enabled in test. */
    public static final Trigger testMode = RobotModeTriggers.test();

    /** True while the robot is disabled. */
    public static final Trigger disabled = RobotModeTriggers.disabled();

    public static final Trigger dsAttached = new Trigger(DriverStation::isDSAttached);
}
