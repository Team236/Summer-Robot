package com.team236.frc2026;

import com.team236.frc2026.subsystems.vision.VisionFieldPoseEstimate;
import com.team236.lib.robot.ConcurrentTimeInterpolatableBuffer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;

/**
 * The {@code RobotState} manages the global tracking of the robot's pose on the field, integrating
 * historical odometry and vision estimates.
 */
public class RobotState {
    public static final double kLogBackTime = 1.0;

    private final Consumer<VisionFieldPoseEstimate> mVisionEstimateConsumer;
    private final ConcurrentTimeInterpolatableBuffer<Pose2d> mFieldToRobot =
            ConcurrentTimeInterpolatableBuffer.createBuffer(kLogBackTime);

    private double mLastUsedMegatagTimestamp = 0;
    private Pose2d mLastUsedMegatagPose = Pose2d.kZero;

    public RobotState(Consumer<VisionFieldPoseEstimate> visionEstimateConsumer) {
        this.mVisionEstimateConsumer = visionEstimateConsumer;
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }

    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        mFieldToRobot.addSample(timestamp, pose);
    }

    public double getLastUsedMegatagTimestamp() {
        return mLastUsedMegatagTimestamp;
    }

    public Optional<Pose2d> getPriorPose(double timestamp) {
        return mFieldToRobot.getSample(timestamp);
    }

    public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
        return mFieldToRobot.getLatest();
    }

    public void updateMegatagEstimate(VisionFieldPoseEstimate megatagEstimate) {
        mLastUsedMegatagTimestamp = megatagEstimate.getTimestampSeconds();
        mLastUsedMegatagPose = megatagEstimate.getVisionRobotPose();
        mVisionEstimateConsumer.accept(megatagEstimate);
    }
}
