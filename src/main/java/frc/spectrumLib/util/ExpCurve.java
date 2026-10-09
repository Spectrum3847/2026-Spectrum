package frc.spectrumLib.util;

// Spectrum 3847
// Based on Code from FRC# 2363

/**
 * Maps an input through an exponential curve. Nothing clamps the output, so offset and scalar can
 * push a result past [-1, 1].
 */
public class ExpCurve extends Curve {
    /** Base of the exponent applied to the input. */
    private double expVal;

    public ExpCurve() {
        setExpVal(1.0);
        setOffset(0.0);
        setScalar(1.0);
        setDeadzone(0.0);
    }

    /**
     * @param expVal base of the exponent applied to the input
     * @param deadzone width of the deadband centred on zero
     */
    public ExpCurve(double expVal, double offset, double scalar, double deadzone) {
        setExpVal(expVal);
        setOffset(offset);
        setScalar(scalar);
        setDeadzone(deadzone);
    }

    /**
     * Applies the stages in order: deadzone, exponent, scalar, offset. Changing the order changes
     * the result.
     *
     * @param input the raw input, usually in [-1, 1]
     */
    @Override
    public double calculate(double input) {
        return calculateOffset(calculateScalar(calculateExpVal(calculateDeadzone(input))));
    }

    /**
     * A normalized exponential that keeps the input's sign and reaches 1.0 at both endpoints. With
     * an expVal of 1.0 the input passes through untouched.
     */
    private double calculateExpVal(double input) {
        if (expVal == 1.0) {
            return input;
        }
        return (Math.pow(expVal, Math.abs(input)) - 1.0) / (expVal - 1.0) * Math.signum(input);
    }

    /** Zero or less is stored as 1.0, which makes the mapping the identity. */
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
