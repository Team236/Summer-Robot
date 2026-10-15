package com.team236.lib.subsystems.cancoder;

import org.littletonrobotics.junction.AutoLog;

/**
 * The {@code CanCoderInputs} class defines the auto-loggable inputs for a CANcoder, representing
 * its hardware state including absolute position and velocity.
 */
@AutoLog
public class CanCoderInputs {
    public double absolutePositionRotations = Double.NaN;
    public double velocityRotations = 0.0;
}
