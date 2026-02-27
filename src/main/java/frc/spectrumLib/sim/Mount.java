package frc.spectrumLib.sim;

public interface Mount {

    public enum MountType {
        LINEAR,
        ARM,
    }

    MountType getMountType();

    double getDisplacementX();

    double getDisplacementY();

    double getAngle();

    double getMountX();

    double getMountY();
}
