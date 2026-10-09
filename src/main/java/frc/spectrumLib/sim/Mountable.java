package frc.spectrumLib.sim;

import frc.spectrumLib.sim.Mount.MountType;

/**
 * Mixin interface for a simulated component that attaches to a {@link Mount}. The default helpers
 * place the component on the canvas from the parent mount's type, position, and angle.
 */
public interface Mountable {

    /**
     * Full-geometry placement. The {@link MountedConfig} overloads pass a config's values straight
     * through to this. All lengths are in metres and angles in radians.
     *
     * @return updated X position of the component on the canvas
     */
    default double getUpdatedX(
            MountType mountType,
            double initialX,
            double initialY,
            double initMountX,
            double initMountY,
            double initMountAngle,
            double mountX,
            double mountY,
            double displacementX,
            double displacementY,
            double mountAngle) {
        double radius = getDistance(initialX, initialY, initMountX, initMountY);
        double angle =
                mountAngle
                        + getAngleOffset(
                                initialX, initialY, initMountX, initMountY, initMountAngle);
        // A linear mount carries the component by its displacement; an arm pivots it about the
        // mount's current position.
        double base = mountType == MountType.LINEAR ? initMountX + displacementX : mountX;
        return getXWithAngle(radius, angle, base);
    }

    /**
     * Full-geometry placement. The {@link MountedConfig} overloads pass a config's values straight
     * through to this. All lengths are in metres and angles in radians.
     *
     * @return updated Y position of the component on the canvas
     */
    default double getUpdatedY(
            MountType mountType,
            double initialX,
            double initialY,
            double initMountX,
            double initMountY,
            double initMountAngle,
            double mountX,
            double mountY,
            double displacementX,
            double displacementY,
            double mountAngle) {
        double radius = getDistance(initialX, initialY, initMountX, initMountY);
        double angle =
                mountAngle
                        + getAngleOffset(
                                initialX, initialY, initMountX, initMountY, initMountAngle);
        double base = mountType == MountType.LINEAR ? initMountY + displacementY : mountY;
        return getYWithAngle(radius, angle, base);
    }

    /**
     * The mount and starting geometry a mounted sim config carries. {@link RollerConfig}, {@link
     * ArmConfig} and {@link LinearConfig} all supply these.
     */
    interface MountedConfig {
        /** The parent mount. */
        Mount getMount();

        /** Component's initial canvas X, in metres. */
        double getInitialX();

        /** Component's initial canvas Y, in metres. */
        double getInitialY();

        /** Mount's X at simulation start, in metres. */
        double getInitMountX();

        /** Mount's Y at simulation start, in metres. */
        double getInitMountY();

        /** Mount's angle at simulation start, in radians. */
        double getInitMountAngle();
    }

    /**
     * Placement using the geometry a mounted sim config carries.
     *
     * @return updated X position on the canvas, in metres
     */
    default double getUpdatedX(MountedConfig config) {
        Mount mount = config.getMount();
        return getUpdatedX(
                mount.getMountType(),
                config.getInitialX(),
                config.getInitialY(),
                config.getInitMountX(),
                config.getInitMountY(),
                config.getInitMountAngle(),
                mount.getMountX(),
                mount.getMountY(),
                mount.getDisplacementX(),
                mount.getDisplacementY(),
                mount.getAngle());
    }

    /**
     * Placement using the geometry a mounted sim config carries.
     *
     * @return updated Y position on the canvas, in metres
     */
    default double getUpdatedY(MountedConfig config) {
        Mount mount = config.getMount();
        return getUpdatedY(
                mount.getMountType(),
                config.getInitialX(),
                config.getInitialY(),
                config.getInitMountX(),
                config.getInitMountY(),
                config.getInitMountAngle(),
                mount.getMountX(),
                mount.getMountY(),
                mount.getDisplacementX(),
                mount.getDisplacementY(),
                mount.getAngle());
    }

    /**
     * Angle of a component's initial position relative to its mount, measured off the mount's
     * starting angle. A component left of the mount takes the mirrored branch.
     */
    static double getAngleOffset(
            double initialX, double initialY, double mountX, double mountY, double startingAngle) {
        double hypotenuse = getDistance(initialX, initialY, mountX, mountY);
        if (initialX >= mountX) {
            return Math.asin((initialY - mountY) / hypotenuse) - startingAngle;
        } else {
            return Math.toRadians(180)
                    - Math.asin((initialY - mountY) / hypotenuse)
                    - startingAngle;
        }
    }

    /** Straight-line distance between two canvas points, in metres. */
    static double getDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
    }

    /**
     * Cartesian {@code X} of a point {@code radius} from the reference point, {@code angle} radians
     * off the Y axis. Lengths are in metres.
     */
    static double getXWithAngle(double radius, double angle, double displacementX) {
        return radius * Math.cos(angle) + displacementX;
    }

    /**
     * Cartesian {@code Y} of a point {@code radius} from the reference point, {@code angle} radians
     * off the X axis. Lengths are in metres.
     */
    static double getYWithAngle(double radius, double angle, double displacementY) {
        return radius * Math.sin(angle) + displacementY;
    }
}
