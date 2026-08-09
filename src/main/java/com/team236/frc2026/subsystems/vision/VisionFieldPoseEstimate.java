package com.team236.frc2026.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * The {@code VisionFieldPoseEstimate} represents a pose estimate of the robot on the field provided
 * by the vision system.
 */
public class VisionFieldPoseEstimate {
    private final Pose2d mVisionRobotPose;
    private final double mTimestampSeconds;
    private final Matrix<N3, N1> mVisionMeasurementStdDevs;
    private final int mTagCount;

    public VisionFieldPoseEstimate(
            Pose2d visionRobotPose,
            double timestamp,
            Matrix<N3, N1> visionMeasurementStdDevs,
            int tagCount) {
        this.mVisionRobotPose = visionRobotPose;
        this.mTimestampSeconds = timestamp;
        this.mVisionMeasurementStdDevs = visionMeasurementStdDevs;
        this.mTagCount = tagCount;
    }

    public Pose2d getVisionRobotPose() {
        return mVisionRobotPose;
    }

    public double getTimestampSeconds() {
        return mTimestampSeconds;
    }

    public Matrix<N3, N1> getVisionMeasurementStdDevs() {
        return mVisionMeasurementStdDevs;
    }

    public int getTagCount() {
        return mTagCount;
    }
}
