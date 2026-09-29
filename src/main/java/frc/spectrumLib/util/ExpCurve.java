package frc.spectrumLib.util;

// Spectrum 3847
// Based on Code from FRC# 2363

/**
 * Maps a stick input in [-1, 1] through an exponential of base {@code expVal}, keeping the result
 * in [-1, 1] and preserving sign. The scalar and offset applied after the exponent set the range of
 * the final value.
 */
public class ExpCurve extends Curve {
    private double expVal;

    public ExpCurve() {
        setExpVal(1.0);
        setOffset(0.0);
        setScalar(1.0);
        setDeadzone(0.0);
    }

    public ExpCurve(double expVal, double offset, double scalar, double deadzone) {
        setExpVal(expVal);
        setOffset(offset);
        setScalar(scalar);
        setDeadzone(deadzone);
    }

    /**
     * Applies the deadzone, then the exponent, then the scalar, then the offset. The deadzone is
     * the innermost call, so it runs first.
     */
    @Override
    public double calculate(double input) {
        double val = calculateOffset(calculateScalar(calculateExpVal(calculateDeadzone(input))));
        return val;
    }

    /**
     * Applies a signed power curve of base {@code expVal}. A base above 1 gives fine control near
     * the middle of the range, a base below 1 near the ends. Both ends still land on -1 and 1, so
     * the scalar keeps its meaning as the maximum output.
     */
    private double calculateExpVal(double input) {
        double val = input;
        if (expVal != 1.0) {
            val = (Math.pow(expVal, Math.abs(input)) - 1.0) / (expVal - 1.0) * Math.signum(input);
        }
        return val;
    }

    /**
     * A base at or below zero falls back to 1.0, leaving the input uncurved. A negative base would
     * raise a fractional power and produce imaginary results.
     */
    public void setExpVal(double expVal) {
        if (expVal <= 0.0) {
            expVal = 1.0;
        }
        this.expVal = expVal;
    }

    public double getExpVal() {
        return expVal;
    }
}
