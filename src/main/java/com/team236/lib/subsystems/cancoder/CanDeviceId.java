package com.team236.lib.subsystems.cancoder;

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
        return (otherDevice.getDeviceId() == mDeviceId && otherDevice.getDeviceNetwork() == mNetwork);
    }
}
