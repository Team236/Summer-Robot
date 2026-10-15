package com.team236.lib.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team236.lib.subsystems.cancoder.CanDeviceId;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code CanStatusLogger} tracks and logs the connection status of CAN devices across the robot
 * network to AdvantageKit and SmartDashboard.
 */
public class CanStatusLogger {
    private static CanStatusLogger mInstance;
    private final List<DeviceStatusInfo> mDevices = new ArrayList<>();
    private BaseStatusSignal[] mSignals;

    private CanStatusLogger() {}

    public static synchronized CanStatusLogger getInstance() {
        if (mInstance == null) {
            mInstance = new CanStatusLogger();
        }
        return mInstance;
    }

    public void registerTalonFX(String name, TalonFX talon, int deviceId, String canBus) {
        mDevices.add(new DeviceStatusInfo(name, talon, deviceId, canBus));
    }

    public void registerTalonFX(String name, TalonFX talon, CanDeviceId deviceId) {
        registerTalonFX(name, talon, deviceId.getDeviceId(), deviceId.getDeviceNetwork());
    }

    public void registerCANcoder(String name, CANcoder cancoder, int deviceId, String canBus) {
        // This method is kept for compatibility but doesn't need to track CANcoders
    }

    public void updateCanStatus() {
        if (mSignals == null) {
            initializeSignalArray();
        }
        try {
            BaseStatusSignal.refreshAll(mSignals);
        } catch (Exception e) {
        }

        // Check each device status
        for (int i = 0; i < mDevices.size(); i++) {
            DeviceStatusInfo device = mDevices.get(i);
            String deviceName = device.mName + "ID" + device.mDeviceId;
            boolean isConnected = false;

            if (device.mTalon != null && device.mSupplyVoltage != null) {
                isConnected = (device.mSupplyVoltage.getStatus() == StatusCode.OK);
            }

            Logger.recordOutput("CANstatus/" + deviceName, isConnected);
            SmartDashboard.putBoolean("CAN/" + device.mCanBus + "/" + deviceName, isConnected);
        }
    }

    private void initializeSignalArray() {
        int validSignalCount = 0;
        for (DeviceStatusInfo device : mDevices) {
            if (device.mSupplyVoltage != null) {
                validSignalCount++;
            }
        }

        if (mSignals == null || mSignals.length != validSignalCount) {
            mSignals = new BaseStatusSignal[validSignalCount];
            int index = 0;
            for (DeviceStatusInfo device : mDevices) {
                if (device.mSupplyVoltage != null) {
                    mSignals[index++] = device.mSupplyVoltage;
                }
            }
        }
    }

    /**
     * The {@code DeviceStatusInfo} holds hardware references and network telemetry signals for a
     * specific CAN device.
     */
    private static class DeviceStatusInfo {
        private final String mName;
        private final TalonFX mTalon;
        private final int mDeviceId;
        private final String mCanBus;
        private final StatusSignal<?> mSupplyVoltage;

        public DeviceStatusInfo(String name, TalonFX talon, int deviceId, String canBus) {
            this.mName = name;
            this.mTalon = talon;
            this.mDeviceId = deviceId;
            this.mCanBus = canBus;
            this.mSupplyVoltage = talon.getSupplyVoltage();

            if (this.mSupplyVoltage != null) {
                this.mSupplyVoltage.setUpdateFrequency(100);
            }
        }
    }
}
