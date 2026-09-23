package frc.spectrumLib.sim;

import frc.spectrumLib.sim.Mount.MountType;

/**
 * Mixin interface for simulation components that can be attached to a {@link Mount}. Provides
 * default geometry helpers that compute the component's updated canvas position each simulation
 * period, taking the parent mount's type, current position, and current angle into account.
 */
public interface Mountable {

    /**
     * Computes the updated X position of a mounted component given full explicit geometry
     * parameters.
     *
     * @param mountType type of the parent mount (ARM or LINEAR)
     * @param initialX component's initial X position on the canvas (metres)
     * @param initialY component's initial Y position on the canvas (metres)
     * @param initMountX mount's X position at simulation start (metres)
     * @param initMountY mount's Y position at simulation start (metres)
     * @param initMountAngle mount's angle at simulation start (radians)
     * @param mountX mount's current X position (metres)
     * @param mountY mount's current Y position (metres)
     * @param displacementX mount's current horizontal displacement from its initial position
     *     (metres)
     * @param displacementY mount's current vertical displacement from its initial position (metres)
     * @param mountAngle mount's current angle (radians)
     * @return updated X position of the component on the canvas (metres)
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
     * Computes the updated Y position of a mounted component given full explicit geometry
     * parameters.
     *
     * @param mountType type of the parent mount (ARM or LINEAR)
     * @param initialX component's initial X position on the canvas (metres)
     * @param initialY component's initial Y position on the canvas (metres)
     * @param initMountX mount's X position at simulation start (metres)
     * @param initMountY mount's Y position at simulation start (metres)
     * @param initMountAngle mount's angle at simulation start (radians)
     * @param mountX mount's current X position (metres)
     * @param mountY mount's current Y position (metres)
     * @param displacementX mount's current horizontal displacement from its initial position
     *     (metres)
     * @param displacementY mount's current vertical displacement from its initial position (metres)
     * @param mountAngle mount's current angle (radians)
     * @return updated Y position of the component on the canvas (metres)
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
     * ArmConfig} and {@link LinearConfig} all provide these getters.
     */
    interface MountedConfig {
        /** The parent mount. */
        Mount getMount();

        /** Component's initial X position on the canvas (metres). */
        double getInitialX();

        /** Component's initial Y position on the canvas (metres). */
        double getInitialY();

        /** Mount's X position at simulation start (metres). */
        double getInitMountX();

        /** Mount's Y position at simulation start (metres). */
        double getInitMountY();

        /** Mount's angle at simulation start (radians). */
        double getInitMountAngle();
    }

    /**
     * Convenience overload that derives all geometry parameters from a mounted sim config.
     *
     * @param config the config carrying mount and initial-position data
     * @return updated X position on the canvas (metres)
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
     * Convenience overload that derives all geometry parameters from a mounted sim config.
     *
     * @param config the config carrying mount and initial-position data
     * @return updated Y position on the canvas (metres)
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
     * Returns the radians a mounted object should be away from a mount based on their initial
     * positions
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

    /**
     * Computes the straight-line distance between two points on the canvas.
     *
     * @param x1 X coordinate of the first point (metres)
     * @param y1 Y coordinate of the first point (metres)
     * @param x2 X coordinate of the second point (metres)
     * @param y2 Y coordinate of the second point (metres)
     * @return Euclidean distance in metres
     */
    static double getDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
    }

    /**
     * Computes the X coordinate of a point at {@code radius} distance from {@code displacementX} in
     * the direction of {@code angle}.
     *
     * @param radius distance from the reference point (metres)
     * @param angle direction angle in radians
     * @param displacementX reference X coordinate (metres)
     * @return resulting X coordinate (metres)
     */
    static double getXWithAngle(double radius, double angle, double displacementX) {
        return radius * Math.cos(angle) + displacementX;
    }

    /**
     * Computes the Y coordinate of a point at {@code radius} distance from {@code displacementY} in
     * the direction of {@code angle}.
     *
     * @param radius distance from the reference point (metres)
     * @param angle direction angle in radians
     * @param displacementY reference Y coordinate (metres)
     * @return resulting Y coordinate (metres)
     */
    static double getYWithAngle(double radius, double angle, double displacementY) {
        return radius * Math.sin(angle) + displacementY;
    }
}
