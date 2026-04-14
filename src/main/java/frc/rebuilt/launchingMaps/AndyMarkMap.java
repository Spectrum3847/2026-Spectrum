package frc.rebuilt.launchingMaps;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import lombok.Getter;

public class AndyMarkMap {

    @Getter
    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    @Getter
    private static final InterpolatingDoubleTreeMap launcherSpeedMap =
            new InterpolatingDoubleTreeMap();

    static {
        hoodAngleMap.put(1.34, 16.0);
        hoodAngleMap.put(2.00, 17.0);
        hoodAngleMap.put(2.35, 19.0);
        hoodAngleMap.put(2.65, 20.0);
        hoodAngleMap.put(2.96, 22.0);
        hoodAngleMap.put(3.23, 24.0);
        hoodAngleMap.put(3.65, 25.0);
        hoodAngleMap.put(4.00, 27.0);
        hoodAngleMap.put(4.20, 29.0);
        hoodAngleMap.put(4.50, 31.0);
        hoodAngleMap.put(5.60, 32.0);

        /* Flywheel map (in RPM) */
        // Near Trench
        launcherSpeedMap.put(0.00, 1800.0);
        launcherSpeedMap.put(3.30, 1800.0);

        // Near Tower
        launcherSpeedMap.put(3.31, 1900.0);
        launcherSpeedMap.put(5.00, 1900.0);

        // Feeding shots
        launcherSpeedMap.put(5.01, 2400.0);
        launcherSpeedMap.put(6.00, 2400.0);
    }
}
