package frc.lib.util;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/**
 * Utility class for getting the MAC address of the RoboRIO and determining the robot's identity.
 *
 * @author 2910 and 1678
 */
public class MacAddressUtil {
    // public static final byte[] SANDSPIT2 =
    //         new byte[] {(byte) 0x5a, (byte) 0x56, (byte) 0x16, (byte) 0x37, (byte) 0x7e, (byte) 0x29};
    public static final String SANDSPIT2 = "00:80:2F:41:64:AD";

    /**
     * Gets the MAC address of the RoboRIO.
     *
     * @return the MAC address of the RoboRIO
     * @throws SocketException if no MAC address is found
     */
    public static byte[] getMacAddress() throws SocketException {
        Enumeration<NetworkInterface> networkInterface = NetworkInterface.getNetworkInterfaces();

        NetworkInterface temp;

        while (networkInterface.hasMoreElements()) {
            temp = networkInterface.nextElement();
            if (!temp.getDisplayName().equals("eth0"))
                continue; // gets a consistent MAC address, since the RIO has multiple network interfaces
            byte[] mac = temp.getHardwareAddress();
            if (mac != null) {
                return mac;
            }
        }
        return null;
    }

    /**
     * Converts a raw MAC address byte array to a string.
     *
     * @param mac the MAC address byte array
     * @return the MAC address as a string
     */
    public static String macToString(byte[] mac) {
        if (mac == null) return "null";
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < mac.length; i++) {
            sb.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
        }
        return sb.toString();
    }

    /**
     * Enum representing the possible robot identities.
     */
    public enum RobotIdentity {
        SANDSPIT2,
        SANDSPIT;

        /**
         * Gets the robot identity based on the MAC address.
         *
         * @param mac the MAC address
         * @return the robot identity
         */
        public static RobotIdentity getRobotIdentity(byte[] mac) {
            if (macToString(mac).equals(SANDSPIT2)) {
                // if (Arrays.compare(mac, MacAddressUtil.SANDSPIT2) == 0) {
                return SANDSPIT2;
            } else { // todo: add other robots back in
                System.out.println("Unknown MAC Address: " + macToString(mac));
                System.out.println("Assuming Comp Bot");
                return SANDSPIT;
            }
        }
    }
}
