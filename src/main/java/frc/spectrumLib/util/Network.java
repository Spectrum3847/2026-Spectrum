package frc.spectrumLib.util;

import edu.wpi.first.wpilibj.DriverStation;
import java.net.*;

/** Common Network Utilities */
public class Network {
    static final String unkown = "UNKNOWN";
    /**
     * Gets the MAC address of the robot
     *
     * @return the MAC address of the robot
     */
    public static String getMACaddress() {
        InetAddress localHost;
        NetworkInterface ni;
        byte[] hardwareAddress;
        String mac = "";
        for (int i = 0; i < 10; i++) {
            try {
                localHost = InetAddress.getLocalHost();
                if (localHost == null) return unkown;
                ni = NetworkInterface.getByInetAddress(localHost);
                if (ni == null) return unkown;
                hardwareAddress = ni.getHardwareAddress();
                if (hardwareAddress == null) return unkown;

                String[] hexadecimal = new String[hardwareAddress.length];
                for (int j = 0; j < hardwareAddress.length; j++) {
                    hexadecimal[j] = String.format("%02X", hardwareAddress[j]);
                }
                mac = String.join(":", hexadecimal);
                return mac;
            } catch (UnknownHostException | SocketException e) {
                DriverStation.reportWarning("Failed to get MAC, retrying", null);
            }
        }

        DriverStation.reportWarning("Failed to get MAC", null);
        return unkown;
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
        return unkown;
    }

    /**
     * Gets the IP Address of the device at the address such as "limelight.local"
     *
     * @return the IP Address of the device
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
        return unkown;
    }
}
