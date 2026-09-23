package frc.spectrumLib.util;

import edu.wpi.first.wpilibj.DriverStation;
import java.net.*;
import java.util.HexFormat;

/** Common Network Utilities */
public class Network {
    static final String unknown = "UNKNOWN";
    /**
     * Gets the MAC address of the robot
     *
     * @return the MAC address of the robot
     */
    public static String getMACaddress() {
        InetAddress localHost;
        NetworkInterface ni;
        byte[] hardwareAddress;
        for (int i = 0; i < 10; i++) {
            try {
                localHost = InetAddress.getLocalHost();
                if (localHost == null) return unknown;
                ni = NetworkInterface.getByInetAddress(localHost);
                if (ni == null) return unknown;
                hardwareAddress = ni.getHardwareAddress();
                if (hardwareAddress == null) return unknown;

                return HexFormat.ofDelimiter(":").withUpperCase().formatHex(hardwareAddress);
            } catch (UnknownHostException | SocketException e) {
                DriverStation.reportWarning("Failed to get MAC, retrying", null);
            }
        }

        DriverStation.reportWarning("Failed to get MAC", null);
        return unknown;
    }

    /**
     * Gets the IP address of the robot
     *
     * @return the IP address of the robot
     */
    public static String getIPaddress() {
        InetAddress localHost;
        String ip = "";
        for (int i = 0; i < 10; i++) {
            try {
                localHost = InetAddress.getLocalHost();
                ip = localHost.getHostAddress();
                return ip;
            } catch (UnknownHostException e) {
                DriverStation.reportWarning("Failed to get IP, retrying", null);
            }
        }

        DriverStation.reportWarning("Failed to get IP", null);
        return unknown;
    }

    /**
     * Resolves and returns the IP address of a device identified by its mDNS or hostname address
     * (e.g. {@code "limelight.local"}).
     *
     * @param deviceNameAddress the hostname or mDNS name to resolve
     * @return the resolved IP address string, or {@code "UNKNOWN"} if resolution fails
     */
    public static String getIPaddress(String deviceNameAddress) {
        InetAddress localHost;
        String ip = "";
        for (int i = 0; i < 10; i++) {
            try {
                localHost = InetAddress.getByName(deviceNameAddress);
                ip = localHost.getHostAddress();
                return ip;
            } catch (UnknownHostException e) {
                DriverStation.reportWarning("Failed to get IP, retrying", null);
            }
        }
        DriverStation.reportWarning("Failed to get IP", null);
        return unknown;
    }
}
