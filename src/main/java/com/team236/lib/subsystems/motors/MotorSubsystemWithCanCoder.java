package com.team236.lib.subsystems.motors;

import com.team236.lib.subsystems.cancoder.CanCoderIO;
import com.team236.lib.subsystems.cancoder.CanCoderInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code MotorSubsystemWithCanCoder} extends the base motor subsystem to integrate CANcoder
 * telemetry and handle automatic absolute position offsetting.
 */
public class MotorSubsystemWithCanCoder<
                T extends MotorInputsAutoLogged,
                U extends MotorIO,
                V extends CanCoderInputsAutoLogged,
                W extends CanCoderIO>
        extends MotorSubsystem<T, U> {
    private MotorSubsystemWithCanCoderConfig mConf;
    private V mCancoderInputs;
    private W mCancoderIO;
    private boolean mHasSetOffset = false;

    public MotorSubsystemWithCanCoder(
            MotorSubsystemWithCanCoderConfig config, T inputs, U io, V cancoderInputs, W cancoder) {
        super(config, inputs, io);
        this.mConf = config;
        this.mCancoderInputs = cancoderInputs;
        this.mCancoderIO = cancoder;
    }

    @Override
    public void periodic() {
        super.periodic();

        mCancoderIO.readInputs(mCancoderInputs);
        Logger.processInputs(getName() + "/cancoder", mCancoderInputs);

        if (!this.mConf.isFusedCancoder
                && !this.mHasSetOffset
                && !Double.isNaN(mCancoderInputs.absolutePositionRotations)) {
            io.setCurrentPosition(
                    mCancoderInputs.absolutePositionRotations * mConf.cancoderToUnitsRatio);
            this.mHasSetOffset = true;
        }
    }

    public void resetOffset() {
        // Don't set boolean above to left main thread still do it as well.
        io.setCurrentPosition(
                mCancoderInputs.absolutePositionRotations * mConf.cancoderToUnitsRatio);
    }
}
