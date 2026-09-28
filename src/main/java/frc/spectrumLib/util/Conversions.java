package frc.spectrumLib.util;

import java.util.function.DoubleSupplier;

public class Conversions {

    public static double RPMtoRPS(double rpm) {
        return rpm / 60;
    }

    public static Double RPMtoRPS(DoubleSupplier rpm) {
        return rpm.getAsDouble() / 60;
    }

    public static double RPStoRPM(double rps) {
        return rps * 60;
    }
}
