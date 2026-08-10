package com.team236.frc2026;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class RobotState {
    public double lastUsedMegatagTimestamp = 0;

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }

    public double getLastUsedMegatagTimestamp() {
        return lastUsedMegatagTimestamp;
    }
}
