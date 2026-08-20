package com.team236.frc2026;

import com.team236.frc2026.subsystems.vision.VisionFieldPoseEstimate;
import com.team236.lib.robot.ConcurrentTimeInterpolatableBuffer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import java.util.function.Consumer;

public class RobotState {
    private final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer;

    public double lastUsedMegatagTimestamp = 0;
    private Pose2d lastUsedMegatagPose = Pose2d.kZero;

    public final static double kLogBackTime = 1.0;

    private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot =
            ConcurrentTimeInterpolatableBuffer.createBuffer(kLogBackTime);

    public RobotState(Consumer<VisionFieldPoseEstimate> visionEstimateConsumer) {
        this.visionEstimateConsumer = visionEstimateConsumer;
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }

    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        fieldToRobot.addSample(timestamp, pose);
    }

    public double getLastUsedMegatagTimestamp() {
        return lastUsedMegatagTimestamp;
    }

    public Optional<Pose2d> getPriorPose(double timestamp) {
        return fieldToRobot.getSample(timestamp);
    }

    public void updateMegatagEstimate(VisionFieldPoseEstimate megatagEstimate) {
        lastUsedMegatagTimestamp = megatagEstimate.getTimestampSeconds();
        lastUsedMegatagPose = megatagEstimate.getVisionRobotPose();
        visionEstimateConsumer.accept(megatagEstimate);
    }
}
