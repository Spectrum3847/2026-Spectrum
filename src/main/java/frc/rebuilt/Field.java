// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.rebuilt;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class Field {
    @Getter public static final double fieldLength = Units.inchesToMeters(690.876);
    @Getter private static final double halfLength = fieldLength / 2.0;
    @Getter public static final double fieldWidth = Units.inchesToMeters(317);
    @Getter private static final double halfWidth = fieldWidth / 2.0;

    @Getter
    private static final Pose2d centerField = new Pose2d(halfLength, halfWidth, new Rotation2d());

    @Getter
    public static final double startingLineX =
            Units.inchesToMeters(299.438); // Measured from the inside of starting line

    @Getter public static final Translation3d blueHubCenter = new Translation3d(
            Units.inchesToMeters(182.11),
            Units.inchesToMeters(158.84),
            Units.inchesToMeters(72));
    @Getter public static final Translation3d redHubCenter = new Translation3d(
            Units.inchesToMeters(469.11),
            Units.inchesToMeters(158.84),
            Units.inchesToMeters(72));

    @Getter private static final double aprilTagWidth = Units.inchesToMeters(6.50);


    /** Returns {@code true} if the robot is on the blue alliance. */
    public static boolean isBlue() {
        return DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .equals(DriverStation.Alliance.Blue);
    }

    /** Returns {@code true} if the robot is on the red alliance. */
    public static boolean isRed() {
        return !isBlue();
    }

    public static final Trigger red = new Trigger(Field::isRed);
    public static final Trigger blue = new Trigger(Field::isBlue);
}
