package frc.spectrumLib.sim;

/**
 * A simulation component other mechanisms attach to. Implementations expose their current position
 * and angle, so a child can place itself once per simulation period.
 */
public interface Mount {

    public enum MountType {
        /** A linear (elevator-style) stage mount. */
        LINEAR,
        /** A single-jointed arm mount. */
        ARM,
    }

    /** Children switch on this to choose between linear and arm placement. */
    MountType getMountType();

    /** Horizontal distance from the initial position, in metres. */
    double getDisplacementX();

    /** Vertical distance from the initial position, in metres. */
    double getDisplacementY();

    /** Current absolute angle, in radians. */
    double getAngle();

    /** The X coordinate a child should attach to, in metres. */
    double getMountX();

    /** The Y coordinate a child should attach to, in metres. */
    double getMountY();
}
