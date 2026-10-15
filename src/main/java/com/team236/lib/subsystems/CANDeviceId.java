package com.team236.lib.subsystems;

public class CANDeviceId {
    private final int deviceId;
    private final String network;

    public CANDeviceId(int deviceId, String network) {
        this.deviceId = deviceId;
        this.network = network;
    }

    public CANDeviceId(int deviceId) {
        this(deviceId, "");
    }

    public int getDeviceId() {
        return deviceId;
    }

    public String getDeviceNetwork() {
        return network;
    }

    public boolean equals(CANDeviceId otherDevice) {
        return (otherDevice.deviceId == deviceId && otherDevice.network == network);
    }
}
