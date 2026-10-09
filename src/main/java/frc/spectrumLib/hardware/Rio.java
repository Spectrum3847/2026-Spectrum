package frc.spectrumLib.hardware;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.spectrumLib.telemetry.Telemetry;
import java.util.HashMap;
import java.util.Map;

/**
 * Identifies the roboRIO that is running, keyed by its serial number, so a robot-specific config
 * can be selected at startup.
 *
 * <p>Serial numbers are on the label on the back of the roboRIO, and a short one needs a leading
 * zero. Reflashing the roboRIO can change the serial number.
 *
 * <p>Based on:
 * https://github.com/Team100/all24/blob/2a109b28467cfddcafb93c7fc85ef60b56a628a2/lib/src/main/java/org/team100/lib/config/Identity.java
 */
public enum Rio {
    PHOTON2026("032B4BB3", true),
    PM_2026("0329AD07", true),
    FM_2026("", true),
    OM_2026("", true),

    FM_2025("0329F2D1", true),

    FM_2024("032B1F69", true),

    SIM("", true), // the only constant allowed a blank serial, so it claims the simulator
    UNKNOWN(null, true);

    private static final Map<String, Rio> IDs = new HashMap<>();

    static {
        for (Rio i : Rio.values()) {
            // A blank serial is a placeholder for a roboRIO nobody has recorded yet. Only SIM may
            // hold one, or whichever blank constant came last would claim the simulator.
            if (i.serialNumber != null && (i == SIM || !i.serialNumber.isEmpty())) {
                IDs.put(i.serialNumber, i);
            }
        }
    }

    private static final Alert rioIdAlert = new Alert("RIO: ", AlertType.kInfo);
    private static final Alert rioIdUnknown = new Alert("UNKNOWN RIO: ", AlertType.kError);
    private static final Alert rio1alert = new Alert("RIO 1.0", AlertType.kWarning);

    /** The constant matching the roboRIO this code is running on. */
    public static final Rio id = checkID();

    /** Bus name that selects the first CANivore the system has. */
    public static final String CANIVORE = "*";
    /** Bus name of the roboRIO's own CAN interface. */
    public static final String RIO_CANBUS = "rio";

    private final String serialNumber;
    private final boolean isRio2;

    private Rio(String serialNumber, boolean isRio2) {
        this.serialNumber = serialNumber;
        this.isRio2 = isRio2;
    }

    private static Rio checkID() {
        String serialNumber = "";
        if (RobotBase.isReal()) {
            // RobotController.getSerialNumber SEGVs under a VS Code unit test, so it only runs
            // on real hardware.
            serialNumber = RobotController.getSerialNumber();
            Telemetry.print("RIO SERIAL: " + serialNumber);
        }

        if (IDs.containsKey(serialNumber)) {
            Rio id = IDs.get(serialNumber);
            rioIdAlert.setText("Rio: " + id.name());
            rioIdAlert.set(true);

            Telemetry.print("RIO NAME: " + id.name());
            if (!id.isRio2) {
                rio1alert.set(true);
            }
            return id;
        }
        rioIdUnknown.setText("Unknown Rio: " + serialNumber);
        rioIdUnknown.set(true);
        return UNKNOWN;
    }

    /** True for a RIO 2.0 controller, false for a RIO 1.0. */
    public boolean isRio2() {
        return isRio2;
    }
}
