package com.team236.lib.subsystems.motor;

import com.team236.lib.subsystems.cancoder.CanCoderConfig;

/**
 * The {@code MotorSubsystemWithCanCoderConfig} class is for a MotorSubsystem utilizing a CANcoder.
 * Stores hardware configuration, unit conversion ratios, and simulation settings.
 */
public class MotorSubsystemWithCanCoderConfig extends MotorSubsystemConfig {

    public CanCoderConfig canCoderConfig = new CanCoderConfig();

    // This is the ratio from cancoder to units.
    // cancoder rotations * by this ratio should = units of subsystem.
    public double cancoderToUnitsRatio = 1.0;

    public boolean isFusedCancoder = false;

    public double ratioForSim = 1.0;

    public double cancoderUnitsForSim = 1.0;

    /**
     * Calculates the ratio from the rotor to the CANcoder. rotor * by this ratio should = cancoder.
     *
     * @return The calculated rotor to CANcoder ratio.
     */
    public double getCancoderToRotorRatio() {
        return unitToRotorRatio / cancoderToUnitsRatio;
    }
}
