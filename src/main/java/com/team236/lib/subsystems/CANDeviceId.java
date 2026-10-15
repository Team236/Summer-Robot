package com.team236.lib.subsystems;

public class CANDeviceId {
    private final int mDeviceId;
    private final String mNetwork;

    public CANDeviceId(int deviceId, String network) {
        this.mDeviceId = deviceId;
        this.mNetwork = network;
    }

    public CANDeviceId(int deviceId) {
        this(deviceId, "");
    }

    public int getDeviceId() {
        return mDeviceId;
    }

    public String getDeviceNetwork() {
        return mNetwork;
    }

    public boolean equals(CANDeviceId otherDevice) {
        return (otherDevice.getDeviceId() == mDeviceId && otherDevice.getDeviceNetwork() == mNetwork);
    }
}
