package frc.spectrumLib.util;

import edu.wpi.first.wpilibj.DriverStation;
import java.net.*;
import java.util.HexFormat;

/**
 * Reads the robot's own addresses and resolves hostnames to IP addresses. Each lookup retries ten
 * times, reporting a warning on every failure, before giving up.
 */
public class Network {
    static final String unknown = "UNKNOWN";
    /**
     * Returns the robot's MAC address as uppercase colon-separated hex, or {@code UNKNOWN} if it
     * cannot be read.
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

    /** Returns the robot's own IP address, or {@code UNKNOWN} if it cannot be resolved. */
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
     * Resolves a hostname or mDNS name such as {@code limelight.local}. Returns {@code UNKNOWN} if
     * the name never resolves.
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
