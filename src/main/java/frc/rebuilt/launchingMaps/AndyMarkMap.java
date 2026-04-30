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
        hoodAngleMap.put(1.85, 9.5);
        hoodAngleMap.put(2.00, 10.5);
        hoodAngleMap.put(2.35, 12.3);
        hoodAngleMap.put(2.55, 13.8);
        hoodAngleMap.put(2.65, 14.5);
        hoodAngleMap.put(2.96, 16.0);
        hoodAngleMap.put(3.30, 19.5);

        hoodAngleMap.put(3.31, 19.6);
        hoodAngleMap.put(3.65, 22.3);
        hoodAngleMap.put(4.00, 24.0);

        hoodAngleMap.put(4.01, 20.5);
        hoodAngleMap.put(4.20, 22.0);
        hoodAngleMap.put(4.50, 22.0);

        hoodAngleMap.put(5.20, 30.0);
        hoodAngleMap.put(5.60, 40.0);

        /* Flywheel map (in RPM) */
        // Near Trench
        launcherSpeedMap.put(0.00, 2000.0);
        launcherSpeedMap.put(3.30, 2000.0);

        // Near Tower
        launcherSpeedMap.put(3.31, 2000.0);
        launcherSpeedMap.put(4.00, 2000.0);

        launcherSpeedMap.put(4.01, 2400.0);
        launcherSpeedMap.put(8.99, 2400.0);

        launcherSpeedMap.put(9.99, 4000.0);
        launcherSpeedMap.put(20.00, 4000.0);
    }
}
