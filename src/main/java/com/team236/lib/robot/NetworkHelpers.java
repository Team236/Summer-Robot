package com.team236.lib.robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/**
 * The {@code NetworkHelpers} provides utility methods for network operations, such as verifying
 * system MAC addresses.
 */
public class NetworkHelpers {

    /**
     * Check if this system has a certain mac address in any network device. (Taken from 254).
     *
     * @param macAddress Mac address to check (Uppercase with colons).
     * @return true if some device with this mac address exists on this system.
     */
    public static boolean hasMacAddress(final String macAddress) {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis == null) {
                    continue;
                }
                StringBuilder deviceMacSb = new StringBuilder();
                System.out.println("hasMacAddress: NIS: " + nis.getDisplayName());
                byte[] mac = nis.getHardwareAddress();
                if (mac != null) {
                    for (int i = 0; i < mac.length; i++) {
                        deviceMacSb.append(
                                String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
                    }
                    String deviceMac = deviceMacSb.toString();
                    System.out.println(
                            "hasMacAddress: NIS "
                                    + nis.getDisplayName()
                                    + " deviceMac: "
                                    + deviceMac);
                    if (macAddress.equals(deviceMac)) {
                        System.out.println("hasMacAddress: ** Mac address match! " + deviceMac);
                        return true;
                    }
                } else {
                    System.out.println("hasMacAddress: Address doesn't exist or is not accessible");
                }
            }

        } catch (SocketException e) {
            e.printStackTrace();
        }
        return false;
    }
}
