// Field geometry adapted from Littleton Robotics' 2026 FieldConstants.
//
// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.rebuilt;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Contains field dimensions, the location of every field element, and other useful reference
 * points.
 *
 * <p>NOTE: All constants are defined relative to the field coordinate system, and from the
 * perspective of the blue alliance station.
 *
 * <p>Reference points are derived from the official AprilTag layout wherever possible rather than
 * being hand-entered, so they stay in the same frame the vision pose estimator reports in.
 */
public class Field {

    public static final FieldType fieldType = FieldType.WELDED;

    // AprilTag related constants
    public static final int aprilTagCount =
            AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
    public static final double aprilTagWidth = Units.inchesToMeters(6.5);
    public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

    // Field dimensions
    @Getter
    public static final double fieldLength =
            AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();

    @Getter
    public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

    public static final Translation2d fieldCenter =
            new Translation2d(fieldLength / 2.0, fieldWidth / 2.0);

    // Fuel dimensions
    public static final double fuelDiameter = Units.inchesToMeters(5.91);

    public static final double trenchBarWidth = Units.inchesToMeters(2.95);

    /**
     * Pose of an official AprilTag. Always reads the OFFICIAL layout so field geometry stays fixed
     * even when vision is pointed at a reduced layout.
     *
     * @param id the tag ID
     * @throws IllegalStateException if the tag is not present in the official layout
     */
    private static Pose3d tagPose(int id) {
        return AprilTagLayoutType.OFFICIAL
                .getLayout()
                .getTagPose(id)
                .orElseThrow(
                        () ->
                                new IllegalStateException(
                                        "AprilTag " + id + " missing from the official layout"));
    }

    /** X of an official AprilTag, in meters. */
    private static double tagX(int id) {
        return tagPose(id).getX();
    }

    /** Y of an official AprilTag, in meters. */
    private static double tagY(int id) {
        return tagPose(id).getY();
    }

    /**
     * Officially defined and relevant vertical lines found on the field (defined by X-axis offset)
     */
    public static class LinesVertical {
        public static final double center = fieldLength / 2.0;
        public static final double starting = tagX(26);
        public static final double allianceZone = starting;
        public static final double hubCenter = Hub.centerX;
        public static final double neutralZoneNear = center - Units.inchesToMeters(120);
        public static final double neutralZoneFar = center + Units.inchesToMeters(120);
        // Midpoint of the opposing hub's two face tags, for the same reason as Hub.centerX
        public static final double oppHubCenter = (tagX(4) + tagX(10)) / 2.0;
        public static final double oppAllianceZone = tagX(10);

        /**
         * The painted starting line, measured from its inside edge. Distinct from {@link #starting}
         * above, which is the alliance zone boundary at the hub's near face.
         */
        public static final double startingLine = Units.inchesToMeters(299.438);
    }

    /**
     * Officially defined and relevant horizontal lines found on the field (defined by Y-axis
     * offset)
     *
     * <p>NOTE: The field element start and end are always left to right from the perspective of the
     * alliance station
     */
    public static class LinesHorizontal {

        public static final double center = fieldWidth / 2.0;

        // Right of hub
        public static final double rightBumpStart = Hub.nearRightCorner.getY();
        public static final double rightBumpEnd = rightBumpStart - RightBump.width;
        public static final double rightBumpMiddle = (rightBumpStart + rightBumpEnd) / 2.0;
        public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
        public static final double rightTrenchOpenEnd = 0;
        public static final double rightTrenchMiddle =
                (rightTrenchOpenStart + rightTrenchOpenEnd) / 2.0;

        // Left of hub
        public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
        public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
        public static final double leftBumpMiddle = (leftBumpStart + leftBumpEnd) / 2.0;
        public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
        public static final double leftTrenchOpenStart = fieldWidth;
        public static final double leftTrenchMiddle =
                (leftTrenchOpenEnd + leftTrenchOpenStart) / 2.0;
    }

    /** Hub related constants */
    public static class Hub {

        // Dimensions
        public static final double width = Units.inchesToMeters(47.0);
        // includes the catcher at the top
        public static final double height = Units.inchesToMeters(72.0);
        public static final double innerWidth = Units.inchesToMeters(41.7);
        public static final double innerHeight = Units.inchesToMeters(56.5);

        /**
         * Hub center X, taken as the midpoint of the near- and far-face tags (26 and 20).
         *
         * <p>Upstream used {@code tagX(26) + width / 2}, which lands 0.27 in short: the tag-to-tag
         * span across the hub is 47.53 in, not the nominal 47.0. That error flipped sign under
         * BlueToRed, so the blue and red hub centers ended up 0.54 in apart.
         */
        public static final double centerX = (tagX(26) + tagX(20)) / 2.0;

        // Relevant reference points on alliance side
        public static final Translation3d topCenterPoint =
                new Translation3d(centerX, fieldWidth / 2.0, height);
        public static final Translation3d innerCenterPoint =
                new Translation3d(centerX, fieldWidth / 2.0, innerHeight);

        public static final Translation2d nearLeftCorner =
                new Translation2d(
                        topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d nearRightCorner =
                new Translation2d(
                        topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
        public static final Translation2d farLeftCorner =
                new Translation2d(
                        topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d farRightCorner =
                new Translation2d(
                        topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

        // Hub faces
        public static final Pose2d nearFace = tagPose(26).toPose2d();
        public static final Pose2d farFace = tagPose(20).toPose2d();
        public static final Pose2d rightFace = tagPose(18).toPose2d();
        public static final Pose2d leftFace = tagPose(21).toPose2d();
    }

    /** Left Bump related constants */
    public static class LeftBump {

        // Dimensions
        public static final double width = Units.inchesToMeters(73.0);
        public static final double height = Units.inchesToMeters(6.513);
        public static final double depth = Units.inchesToMeters(44.4);

        // Relevant reference points on alliance side
        public static final Translation2d nearLeftCorner =
                Hub.nearLeftCorner.plus(new Translation2d(0.0, width));
        public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
        public static final Translation2d farLeftCorner =
                Hub.farLeftCorner.plus(new Translation2d(0.0, width));
        public static final Translation2d farRightCorner = Hub.farLeftCorner;
    }

    /** Right Bump related constants */
    public static class RightBump {
        // Dimensions
        public static final double width = Units.inchesToMeters(73.0);
        public static final double height = Units.inchesToMeters(6.513);
        public static final double depth = Units.inchesToMeters(44.4);

        // Relevant reference points on alliance side
        public static final Translation2d nearLeftCorner = Hub.nearRightCorner;
        public static final Translation2d nearRightCorner =
                Hub.nearRightCorner.minus(new Translation2d(0.0, width));
        public static final Translation2d farLeftCorner = Hub.farRightCorner;
        public static final Translation2d farRightCorner =
                Hub.farRightCorner.minus(new Translation2d(0.0, width));
    }

    /** Left Trench related constants */
    public static class LeftTrench {
        // Dimensions
        public static final double width = Units.inchesToMeters(65.65);
        public static final double depth = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(40.25);
        public static final double openingWidth = Units.inchesToMeters(50.34);
        public static final double openingHeight = Units.inchesToMeters(22.25);

        // Relevant reference points on alliance side
        public static final Translation3d openingTopLeft =
                new Translation3d(LinesVertical.hubCenter, fieldWidth, openingHeight);
        public static final Translation3d openingTopRight =
                new Translation3d(
                        LinesVertical.hubCenter, fieldWidth - openingWidth, openingHeight);
        public static final Translation2d center =
                openingTopLeft
                        .toTranslation2d()
                        .interpolate(openingTopRight.toTranslation2d(), 0.5);
    }

    /** Right Trench related constants */
    public static class RightTrench {

        // Dimensions
        public static final double width = Units.inchesToMeters(65.65);
        public static final double depth = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(40.25);
        public static final double openingWidth = Units.inchesToMeters(50.34);
        public static final double openingHeight = Units.inchesToMeters(22.25);

        // Relevant reference points on alliance side
        public static final Translation3d openingTopLeft =
                new Translation3d(LinesVertical.hubCenter, openingWidth, openingHeight);
        public static final Translation3d openingTopRight =
                new Translation3d(LinesVertical.hubCenter, 0, openingHeight);
        public static final Translation2d center =
                openingTopLeft
                        .toTranslation2d()
                        .interpolate(openingTopRight.toTranslation2d(), 0.5);
    }

    /** Tower related constants */
    public static class Tower {
        // Dimensions
        public static final double width = Units.inchesToMeters(49.25);
        public static final double depth = Units.inchesToMeters(45.0);
        public static final double height = Units.inchesToMeters(78.25);
        public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
        public static final double frontFaceX = Units.inchesToMeters(43.51);

        public static final double uprightHeight = Units.inchesToMeters(72.1);

        // Rung heights from the floor
        public static final double lowRungHeight = Units.inchesToMeters(27.0);
        public static final double midRungHeight = Units.inchesToMeters(45.0);
        public static final double highRungHeight = Units.inchesToMeters(63.0);

        // Relevant reference points on alliance side. The centerline follows tag 31 (y = 147.47
        // in), NOT the field centerline -- the 2026 field has 180-degree rotational symmetry, so
        // the blue and red towers sit at different Y.
        public static final Translation2d centerPoint = new Translation2d(frontFaceX, tagY(31));
        public static final Translation2d leftUpright =
                new Translation2d(
                        frontFaceX, tagY(31) + innerOpeningWidth / 2 + Units.inchesToMeters(0.75));
        public static final Translation2d rightUpright =
                new Translation2d(
                        frontFaceX, tagY(31) - innerOpeningWidth / 2 - Units.inchesToMeters(0.75));
    }

    /** Depot related constants */
    public static class Depot {
        // Dimensions
        public static final double width = Units.inchesToMeters(42.0);
        public static final double depth = Units.inchesToMeters(27.0);
        public static final double height = Units.inchesToMeters(1.125);
        public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

        // Relevant reference points on alliance side. The depot runs from the alliance wall out to
        // `depth`, so its middle is at depth / 2; the corners are the field-side (far) pair.
        public static final Translation3d depotCenter =
                new Translation3d(depth / 2.0, (fieldWidth / 2) + distanceFromCenterY, height);
        public static final Translation3d leftCorner =
                new Translation3d(
                        depth, (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
        public static final Translation3d rightCorner =
                new Translation3d(
                        depth, (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
    }

    /** Outpost related constants */
    public static class Outpost {
        // Dimensions
        public static final double width = Units.inchesToMeters(31.8);
        public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
        public static final double height = Units.inchesToMeters(7.0);

        // Relevant reference points on alliance side
        public static final Translation2d centerPoint = new Translation2d(0, tagY(29));
    }

    /** Fuel pool related constants */
    public static class FuelPool {
        // Dimensions
        public static final double width = Units.inchesToMeters(181.9);
        public static final double depth = Units.inchesToMeters(71.9);

        // Relevant reference points on alliance side
        public static final Translation2d nearLeftCorner =
                new Translation2d(fieldLength / 2.0 - depth / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d nearRightCorner =
                new Translation2d(fieldLength / 2.0 - depth / 2.0, fieldWidth / 2.0 - width / 2.0);
        public static final Translation2d leftCenter =
                new Translation2d(fieldLength / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d rightCenter =
                new Translation2d(fieldLength / 2.0, fieldWidth / 2.0 - width / 2.0);
    }

    // -----------------------------------------------------------------------
    // Feeding reference points
    // -----------------------------------------------------------------------

    public static final Translation2d normalFeedBlueLeft = new Translation2d(1, fieldWidth - 1);
    public static final Translation2d normalFeedBlueRight = new Translation2d(1, 1);
    public static final Translation2d normalFeedRedLeft = new Translation2d(fieldLength - 1, 1);
    public static final Translation2d normalFeedRedRight =
            new Translation2d(fieldLength - 1, fieldWidth - 1);

    public static final Translation2d deepFeedBlueLeft = new Translation2d(1, fieldWidth - 2.5);
    public static final Translation2d deepFeedBlueRight = new Translation2d(1, 2.5);
    public static final Translation2d deepFeedRedLeft = new Translation2d(fieldLength - 1, 2.5);
    public static final Translation2d deepFeedRedRight =
            new Translation2d(fieldLength - 1, fieldWidth - 2.5);

    // -----------------------------------------------------------------------
    // Alliance helpers
    // -----------------------------------------------------------------------

    /**
     * Maps a blue-alliance field point to its red-alliance counterpart.
     *
     * <p>The 2026 field has 180-degree rotational symmetry, not mirror symmetry, so both X and Y
     * flip. Verified against the tag layout: rotating every blue tag lands on a real red tag
     * (16/16), while flipping X alone only works for the 10 that happen to sit symmetrically.
     * Matches {@link FieldHelpers#flipIfRed}.
     */
    public static Translation3d BlueToRed(Translation3d translation) {
        return new Translation3d(
                fieldLength - translation.getX(),
                fieldWidth - translation.getY(),
                translation.getZ());
    }

    /** See {@link #BlueToRed(Translation3d)}. */
    public static Translation2d BlueToRed(Translation2d translation) {
        return new Translation2d(fieldLength - translation.getX(), fieldWidth - translation.getY());
    }

    /** Flips a bare X coordinate to the red side. Y has no meaning here, so nothing else to do. */
    public static double BlueToRed(double translation) {
        return fieldLength - translation;
    }

    /**
     * Holder so the hub centers are computed on first use rather than during {@code
     * Field.<clinit>}.
     *
     * <p>They cannot be plain static fields up here. {@link Hub} reads {@link #tagX}, which forces
     * {@code Field} to initialize; if something touches {@code Field.Hub} before it touches {@code
     * Field}, the JVM would re-enter {@code Hub.<clinit>}, see it already in progress, skip it, and
     * hand back a null {@code topCenterPoint}. Deferring to a holder breaks the cycle: by the time
     * anything calls the getters below, both classes are fully initialized.
     */
    private static final class HubCenters {
        static final Translation3d blue = Hub.topCenterPoint;
        static final Translation3d red = BlueToRed(Hub.topCenterPoint);
    }

    /** Center of the blue hub, at scoring height. */
    public static Translation3d getBlueHubCenter() {
        return HubCenters.blue;
    }

    /** Center of the red hub, at scoring height. */
    public static Translation3d getRedHubCenter() {
        return HubCenters.red;
    }

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

    // -----------------------------------------------------------------------
    // AprilTag layout plumbing
    // -----------------------------------------------------------------------

    /**
     * Which physical variant of the field to load tags for.
     *
     * <p>Littleton also had an {@code HQ} variant backed by its own JSON tree. WPILib only ships
     * the welded and AndyMark layouts, so that option is gone; add a deploy-directory loader here
     * if an HQ layout is ever needed.
     */
    @RequiredArgsConstructor
    public enum FieldType {
        WELDED(AprilTagFields.k2026RebuiltWelded),
        ANDYMARK(AprilTagFields.k2026RebuiltAndymark);

        @Getter private final AprilTagFields wpilibField;
    }

    /**
     * Selectable AprilTag layouts.
     *
     * <p>Littleton had reduced layouts (hub only, outpost only, tower only) loaded from JSON files
     * under {@code deploy/apriltags}. This project has no such files, so only the full official
     * layout and an empty one are available. To add a subset, filter {@code OFFICIAL.getLayout()
     * .getTags()} by ID and build a new {@link AprilTagFieldLayout} from the result.
     */
    public enum AprilTagLayoutType {
        OFFICIAL,
        NONE;

        private volatile AprilTagFieldLayout layout;
        private volatile String layoutString;
        /**
         * Returns the layout.
         *
         * @return the layout
         */
        public AprilTagFieldLayout getLayout() {
            if (layout == null) {
                synchronized (this) {
                    if (layout == null) {
                        // Reading fieldType can force Field.<clinit>, which calls back into this
                        // method and fills in `layout` before control returns here. Re-check so we
                        // don't load and discard a second copy.
                        AprilTagFields wpilibField = fieldType.getWpilibField();
                        if (layout != null) {
                            return layout;
                        }
                        AprilTagFieldLayout official = AprilTagFieldLayout.loadField(wpilibField);
                        layout =
                                this == OFFICIAL
                                        ? official
                                        : new AprilTagFieldLayout(
                                                List.of(),
                                                official.getFieldLength(),
                                                official.getFieldWidth());
                        try {
                            layoutString = new ObjectMapper().writeValueAsString(layout);
                        } catch (Exception e) {
                            throw new RuntimeException(
                                    "Failed to serialize AprilTag layout " + name(), e);
                        }
                    }
                }
            }
            return layout;
        }
        /**
         * Returns the layout string.
         *
         * @return the layout string
         */
        public String getLayoutString() {
            if (layoutString == null) {
                getLayout();
            }
            return layoutString;
        }
    }

    // Constructed last: Trigger touches the CommandScheduler, which requires the HAL.
    public static final Trigger red = new Trigger(Field::isRed);
    public static final Trigger blue = new Trigger(Field::isBlue);
}
