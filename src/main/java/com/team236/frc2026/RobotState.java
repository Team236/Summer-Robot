package com.team236.frc2026;

import com.team236.lib.robot.ConcurrentTimeInterpolatableBuffer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class RobotState {
    public double lastUsedMegatagTimestamp = 0;
    public final double kLogBackTime = 1.0;

    private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot =
            ConcurrentTimeInterpolatableBuffer.createBuffer(kLogBackTime);


    public RobotState(){

    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }

    public double getLastUsedMegatagTimestamp() {
        return lastUsedMegatagTimestamp;
    }

    public Optional<Pose2d> getPriorPose(double timestamp) {
        return fieldToRobot.getSample(timestamp);
    }   
}
