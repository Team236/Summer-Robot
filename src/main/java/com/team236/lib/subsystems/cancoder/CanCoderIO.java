package com.team236.lib.subsystems.cancoder;

/**
 * The {@code CanCoderIO} defines the hardware interaction interface for reading telemetry and
 * configuring update frequencies of a CANcoder.
 */
public interface CanCoderIO {
    void readInputs(CanCoderInputs inputs);

    void updateFrequency(double hz);
}
