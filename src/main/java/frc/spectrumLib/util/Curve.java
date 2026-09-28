package frc.spectrumLib.util;

// Spectrum 3847
// Based on Code from FRC# 2363

/**
 * Remaps a stick input in [-1, 1] by deadzoning it, letting a subclass shape it, then scaling and
 * offsetting it. Instantiate a subclass such as {@link ExpCurve} rather than this class.
 *
 * @author Justin Babilino
 */
public abstract class Curve {
    private double offset;
    private double scalar;

    private double deadzone;

    public abstract double calculate(double input);

    /**
     * Maps input to zero inside a deadband of total width {@code deadzone}, then rescales the rest
     * so the curve still spans [-1, 1].
     */
    protected double calculateDeadzone(double input) {
        double deadRadius = deadzone / 2.0;
        double val = 0.0;
        if (input > deadRadius) {
            val = (1.0 / (1.0 - deadRadius)) * (input - deadRadius);
        } else if (input < -deadRadius) {
            val = (1.0 / (1.0 - deadRadius)) * (input + deadRadius);
        }
        return val;
    }

    protected double calculateScalar(double input) {
        double val = input * scalar;
        return val;
    }

    protected double calculateOffset(double input) {
        double val = input + offset;
        return val;
    }

    /**
     * Samples the curve at {@code pointCount} x values spread evenly over [-1, 1], returning an
     * array of {x, y} pairs.
     */
    public double[][] getCurvePoints(int pointCount) {
        double[][] points = new double[pointCount][2];
        double dx = 2.0 / (pointCount - 1);
        for (int i = 0; i < pointCount; i++) {
            double x = -1.0 + (i * dx);
            points[i][0] = x;
            points[i][1] = calculate(x);
        }
        return points;
    }

    /**
     * Prints (x, y) pairs in a form you can paste into https://www.desmos.com/calculator to plot
     * the curve.
     */
    public void printPoints(double[][] points) {
        System.out.println();
        for (double[] point : points) {
            System.out.print("(" + point[0] + "," + point[1] + ")" + ",");
        }
    }

    public void printPoints(int pointCount) {
        printPoints(getCurvePoints(pointCount));
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public void setScalar(double scalar) {
        this.scalar = scalar;
    }

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
