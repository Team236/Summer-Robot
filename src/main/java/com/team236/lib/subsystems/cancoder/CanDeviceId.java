package com.team236.lib.subsystems.cancoder;

import java.util.Objects;

/**
 * The {@code CanDeviceId} class encapsulates a CAN device's ID and network name to uniquely
 * identify it on the robot's CAN bus system.
 */
public class CanDeviceId {

    private final int mDeviceId;
    private final String mNetwork;

    public CanDeviceId(int deviceId, String network) {
        this.mDeviceId = deviceId;
        this.mNetwork = network;
    }

    public CanDeviceId(int deviceId) {
        this(deviceId, "");
    }

    public int getDeviceId() {
        return mDeviceId;
    }

    public String getDeviceNetwork() {
        return mNetwork;
    }

    public boolean equals(CanDeviceId otherDevice) {
        if (otherDevice == null) {
            return false;
        }
        return (otherDevice.getDeviceId() == mDeviceId
                && Objects.equals(otherDevice.getDeviceNetwork(), mNetwork));
    }
}
