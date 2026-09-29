package frc.spectrumLib.sim;

import frc.spectrumLib.sim.Mount.MountType;

/**
 * Canvas geometry for a simulated component that hangs off a {@link Mount}. The default helpers
 * return the component's position on the canvas, in metres, for the current mount state.
 */
public interface Mountable {

    /**
     * Component X on the canvas, swung off the mount by the distance and angle the two started
     * apart. A linear stage is referenced from its start X plus travel, an arm from its live X.
     *
     * @param mountType the parent mount's type
     * @param initialX component X at the start of the run (m)
     * @param initialY component Y at the start of the run (m)
     * @param initMountX mount X at the start of the run (m)
     * @param initMountY mount Y at the start of the run (m)
     * @param initMountAngle mount angle at the start of the run (rad)
     * @param mountX mount X now (m)
     * @param mountY mount Y now (m)
     * @param displacementX mount's horizontal displacement since the run started (m)
     * @param displacementY mount's vertical displacement since the run started (m)
     * @param mountAngle mount angle now (rad)
     * @return the component's canvas X, in metres
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
        switch (mountType) {
            case LINEAR:
                return getXWithAngle(
                        getDistance(initialX, initialY, initMountX, initMountY),
                        mountAngle
                                + getAngleOffset(
                                        initialX, initialY, initMountX, initMountY, initMountAngle),
                        initMountX + displacementX);
            case ARM:
                return getXWithAngle(
                        getDistance(initialX, initialY, initMountX, initMountY),
                        mountAngle
                                + getAngleOffset(
                                        initialX, initialY, initMountX, initMountY, initMountAngle),
                        mountX);
            default:
                return initialX;
        }
    }

    /**
     * Component Y on the canvas, swung off the mount by the distance and angle the two started
     * apart. A linear stage is referenced from its start Y plus travel, an arm from its live Y.
     *
     * @param mountType the parent mount's type
     * @param initialX component X at the start of the run (m)
     * @param initialY component Y at the start of the run (m)
     * @param initMountX mount X at the start of the run (m)
     * @param initMountY mount Y at the start of the run (m)
     * @param initMountAngle mount angle at the start of the run (rad)
     * @param mountX mount X now (m)
     * @param mountY mount Y now (m)
     * @param displacementX mount's horizontal displacement since the run started (m)
     * @param displacementY mount's vertical displacement since the run started (m)
     * @param mountAngle mount angle now (rad)
     * @return the component's canvas Y, in metres
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
        switch (mountType) {
            case LINEAR:
                return getYWithAngle(
                        getDistance(initialX, initialY, initMountX, initMountY),
                        mountAngle
                                + getAngleOffset(
                                        initialX, initialY, initMountX, initMountY, initMountAngle),
                        initMountY + displacementY);
            case ARM:
                return getYWithAngle(
                        getDistance(initialX, initialY, initMountX, initMountY),
                        mountAngle
                                + getAngleOffset(
                                        initialX, initialY, initMountX, initMountY, initMountAngle),
                        mountY);
            default:
                return initialY;
        }
    }

    default double getUpdatedX(RollerConfig config) {
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

    default double getUpdatedX(ArmConfig config) {
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

    default double getUpdatedX(LinearConfig config) {
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

    default double getUpdatedY(RollerConfig config) {
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

    default double getUpdatedY(ArmConfig config) {
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

    default double getUpdatedY(LinearConfig config) {
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

    /** Angle from the mount to the component, in radians, given where the two started. */
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

    /** X of a point {@code radius} metres from a reference at {@code angle} radians. */
    static double getXWithAngle(double radius, double angle, double displacementX) {
        return radius * Math.cos(angle) + displacementX;
    }

    /** Y of a point {@code radius} metres from a reference at {@code angle} radians. */
    static double getYWithAngle(double radius, double angle, double displacementY) {
        return radius * Math.sin(angle) + displacementY;
    }
}
