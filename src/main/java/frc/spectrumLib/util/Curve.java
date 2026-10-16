package frc.spectrumLib.util;

import edu.wpi.first.math.MathUtil;

// Spectrum 3847
// Based on Code from FRC# 2363

/**
 * Maps controller stick inputs through a curve. Subclass this rather than instantiating Curve
 * directly, for example {@link ExpCurve}.
 *
 * @author Justin Babilino
 * @version 0.0.3
 */
public abstract class Curve {
    private double offset;
    private double scalar;

    private double deadzone;

    /** Maps a stick input, expected in [-1, 1], to the curve's output. */
    public abstract double calculate(double input);

    /** Blanks the middle of the curve, then rescales what is left to fill [-1, 1] again. */
    protected double calculateDeadzone(double input) {
        // applyDeadband takes a half width, so halve the full width we were given.
        return MathUtil.applyDeadband(input, deadzone / 2.0);
    }

    protected double calculateScalar(double input) {
        return input * scalar;
    }

    protected double calculateOffset(double input) {
        return input + offset;
    }

    /**
     * Samples the curve at pointCount evenly spaced inputs across [-1, 1]. With pointCount of 1,
     * the single point is the center of the input range.
     *
     * @return [x, y] pairs, one per sample
     */
    public double[][] getCurvePoints(int pointCount) {
        double[][] points = new double[pointCount][2];
        if (pointCount == 1) {
            points[0][0] = 0.0;
            points[0][1] = calculate(0.0);
            return points;
        }
        double dx = 2.0 / (pointCount - 1);
        for (int i = 0; i < pointCount; i++) {
            double x = -1.0 + (i * dx);
            points[i][0] = x;
            points[i][1] = calculate(x);
        }
        return points;
    }

    /**
     * Prints [x, y] pairs for pasting into <a href="https://www.desmos.com/calculator">Desmos</a>.
     */
    public void printPoints(double[][] points) {
        System.out.println();
        for (double[] point : points) {
            System.out.print("(" + point[0] + "," + point[1] + ")" + ",");
        }
    }

    /** Prints {@link #getCurvePoints(int)} for pasting into Desmos. */
    public void printPoints(int pointCount) {
        printPoints(getCurvePoints(pointCount));
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public void setScalar(double scalar) {
        this.scalar = scalar;
    }

    /**
     * Sets the deadband width centred on zero. A negative width is stored as its absolute value.
     */
    public void setDeadzone(double deadzone) {
        this.deadzone = Math.abs(deadzone);
    }

    public double getOffset() {
        return offset;
    }

    public double getScalar() {
        return scalar;
    }

    public double getDeadzone() {
        return deadzone;
    }
}
