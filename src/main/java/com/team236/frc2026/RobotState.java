package com.team236.frc2026;

import com.team236.frc2026.subsystems.vision.VisionFieldPoseEstimate;
import com.team236.lib.roborio.ConcurrentTimeInterpolatableBuffer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code RobotState} manages the global tracking of the robot's pose on the field, integrating
 * historical odometry and vision estimates.
 */
public class RobotState {
    private final Consumer<VisionFieldPoseEstimate> mVisionEstimateConsumer;
    private final ConcurrentTimeInterpolatableBuffer<Pose2d> mFieldToRobot =
            ConcurrentTimeInterpolatableBuffer.createBuffer(Constants.kLogBackTime);

    private double mLastUsedMegatagTimestamp = 0;
    private Pose2d mLastUsedMegatagPose = Pose2d.kZero;

    private final AtomicReference<Double> armPivotRadians = new AtomicReference<>(0.0);

    public RobotState(Consumer<VisionFieldPoseEstimate> visionEstimateConsumer) {
        this.mVisionEstimateConsumer = visionEstimateConsumer;

        // Initialize component positions
        armPivotRadians.set(0.0);
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

    public void setArmPivotAngleRadians(double radians) {
        armPivotRadians.set(radians);
    }

    public double getArmPivotAngleRadians() {
        return armPivotRadians.get();
    }

    public void updateLogger() {
        Logger.recordOutput("RobotState/ElevatorHeightMeters", getArmPivotAngleRadians());
    }
}
