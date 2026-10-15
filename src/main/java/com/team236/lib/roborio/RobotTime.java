package com.team236.lib.roborio;

import edu.wpi.first.wpilibj.Timer;

/** The {@code RobotTime} provides utility methods for accessing global robot timestamps. */
public class RobotTime {

    public static double getTimestampSeconds() {
        return Timer.getFPGATimestamp();
    }
}
