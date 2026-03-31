package frc.rebuilt.launchingMaps;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import lombok.Getter;

public class HomeMap {

    @Getter
    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    @Getter
    private static final InterpolatingDoubleTreeMap launcherSpeedMap =
            new InterpolatingDoubleTreeMap();

    public HomeMap() {

        /* Hood angle map (in degrees from horizontal) */
        hoodAngleMap.put(1.34, 16.0);
        hoodAngleMap.put(2.00, 17.0);
        hoodAngleMap.put(2.35, 19.0);
        hoodAngleMap.put(2.65, 21.0);
        hoodAngleMap.put(2.96, 23.0);
        hoodAngleMap.put(3.23, 25.0);
        hoodAngleMap.put(3.65, 27.0);
        hoodAngleMap.put(4.00, 29.0);
        hoodAngleMap.put(4.20, 31.0);
        hoodAngleMap.put(4.50, 33.0);
        hoodAngleMap.put(5.60, 34.0);

        /* Flywheel map (in RPM) */
        // Near Trench
        launcherSpeedMap.put(0.00, 1800.0);
        launcherSpeedMap.put(3.50, 1800.0);

        // Near Tower
        launcherSpeedMap.put(3.51, 1900.0);
        launcherSpeedMap.put(4.50, 1900.0);

        // Feeding shots
        launcherSpeedMap.put(4.51, 2300.0);
        launcherSpeedMap.put(6.00, 2400.0);
    }
}
