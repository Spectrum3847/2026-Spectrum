package frc.spectrumLib.util;

// Based on 254-2023 Class
// https://github.com/Team254/FRC-2023-Public/blob/main/src/main/java/com/team254/lib/drivers/CanDeviceId.java
/**
 * Identifies a CAN device by its numeric device ID and the CAN bus name it lives on. Equality and
 * hashing consider both fields, so two instances with the same device number on different buses are
 * treated as distinct.
 *
 * @param deviceNumber the numeric CAN ID assigned to the device
 * @param bus the name of the CAN bus (e.g. {@code "rio"} or {@code "canivore"}), or an empty string
 *     for the default bus
 */
public record CanDeviceId(int deviceNumber, String bus) {

    /**
     * Creates a CAN device identifier on the default CAN bus (empty string).
     *
     * @param deviceNumber the numeric CAN ID assigned to the device
     */
    public CanDeviceId(int deviceNumber) {
        this(deviceNumber, "");
    }

    /**
     * Returns the numeric CAN device ID.
     *
     * @return the device number
     */
    public int getDeviceNumber() {
        return deviceNumber;
    }

    /**
     * Returns the CAN bus name this device is on.
     *
     * @return the bus name, or an empty string for the default bus
     */
    public String getBus() {
        return bus;
    }
}
