package com.team236.lib.subsystems.cancoder;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.team236.lib.util.CTREUtil;
import com.team236.lib.util.CanStatusLogger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class CanCoderHardware implements CanCoderIO {
    protected final CANcoder canCoder;
    protected CanCoderConfig config;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final BaseStatusSignal[] signals;
    double goodValues = 0.0;

    public CanCoderHardware(CanCoderConfig config) {
        this.config = config;

        canCoder = new CANcoder(config.canId.getDeviceId(), config.canId.getDeviceNetwork());

        CTREUtil.applyConfiguration(canCoder, this.config.config);
        positionSignal = canCoder.getAbsolutePosition();
        velocitySignal = canCoder.getVelocity();

        signals = new BaseStatusSignal[] {positionSignal, velocitySignal};

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, signals);

        CanStatusLogger.getInstance()
                .registerCANcoder(
                        "CANcoder_ID" + config.canId.getDeviceId(),
                        canCoder,
                        config.canId.getDeviceId(),
                        config.canId.getDeviceNetwork());
    }

    @Override
    public void updateFrequency(double hz) {
        BaseStatusSignal.setUpdateFrequencyForAll(hz, signals);
    }

    @Override
    public void readInputs(CanCoderInputs inputs) {
        BaseStatusSignal.refreshAll(signals);
        if (Double.isNaN(inputs.absolutePositionRotations)) {
            BaseStatusSignal.waitForAll(10.0, positionSignal, velocitySignal);
            goodValues++;
        }
        if (goodValues < 50) return;

        inputs.absolutePositionRotations = positionSignal.getValue().in(Rotations);
        inputs.velocityRotations = velocitySignal.getValue().in(RotationsPerSecond);
    }
}
