package com.team236.frc2026;

import com.team236.frc2026.subsystems.vision.VisionFieldPoseEstimate;
import com.team236.lib.roborio.ConcurrentTimeInterpolatableBuffer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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

    private final AtomicReference<Double> mArmPivotRadians = new AtomicReference<>(0.0);
    private final AtomicReference<Pose3d> mArmPose = new AtomicReference<>(new Pose3d());
    private final AtomicReference<Boolean> mIntakePossession = new AtomicReference<>(false);

    public RobotState(Consumer<VisionFieldPoseEstimate> visionEstimateConsumer) {
        this.mVisionEstimateConsumer = visionEstimateConsumer;

        // Initialize subsystems
        configureSubsystems();
    }

    private void configureSubsystems() {
        mArmPivotRadians.set(0.0);
        mArmPose.set(new Pose3d());
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
        mArmPivotRadians.set(radians);
    }

    public double getArmPivotAngleRadians() {
        return mArmPivotRadians.get();
    }

    public void setArmPose(Pose3d pose) {
        mArmPose.set(pose);
    }

    public Pose3d getArmPose() {
        return mArmPose.get();
    }

    public void setIntakePossession(boolean hasPiece) {
        mIntakePossession.set(hasPiece);
    }

    public boolean getIntakePossession() {
        return mIntakePossession.get();
    }

    public void updateLogger() {
        Logger.recordOutput("RobotState/ArmPivotRadians", getArmPivotAngleRadians());
    }
}
