package frc.spectrumLib.sim;

/**
 * A simulated component that other mechanisms attach to. A mounted mechanism reads these values
 * each period to work out its own canvas position.
 */
public interface Mount {

    public enum MountType {
        LINEAR,
        ARM,
    }

    MountType getMountType();

    /** Horizontal displacement from where this mount started, in metres. */
    double getDisplacementX();

    /** Vertical displacement from where this mount started, in metres. */
    double getDisplacementY();

    /** Current angle in radians. */
    double getAngle();

    /** Attachment point X in metres. */
    double getMountX();

    /** Attachment point Y in metres. */
    double getMountY();
}
