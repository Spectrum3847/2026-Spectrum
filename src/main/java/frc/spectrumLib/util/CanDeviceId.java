package frc.spectrumLib.util;

// Based on 254-2023 Class
// https://github.com/Team254/FRC-2023-Public/blob/main/src/main/java/com/team254/lib/drivers/CanDeviceId.java
/**
 * Identifies a CAN device by its device number and the bus it lives on. Equality and hashing cover
 * both, so the same device number on two buses stays two distinct devices.
 *
 * @param bus the bus name, such as {@code "rio"} or {@code "canivore"}, or an empty string for the
 *     default bus
 */
public record CanDeviceId(int deviceNumber, String bus) {

    /** Builds an identifier on the default bus, which is the empty string. */
    public CanDeviceId(int deviceNumber) {
        this(deviceNumber, "");
    }

    public int getDeviceNumber() {
        return deviceNumber;
    }

    public String getBus() {
        return bus;
    }
}
