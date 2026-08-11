package com.team236.frc2026.subsystems.vision;

import com.team236.frc2026.Constants;
import com.team236.frc2026.Constants.VisionConstants;
import com.team236.frc2026.RobotState;
import com.team236.lib.time.RobotTime;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code VisionSubsystem} coordinates vision hardware I/O and telemetry logging for targeting
 * and odometry systems.
 */
public class VisionSubsystem extends SubsystemBase {
    private final VisionIO mIo;
    private final RobotState mRobotState;
    private final VisionIO.VisionIOInputs mInputs = new VisionIO.VisionIOInputs();
    private double quality;

    public VisionSubsystem(VisionIO io, RobotState robotState) {
        this.mIo = io;
        this.mRobotState = robotState;
    }

    @Override
    public void periodic() {
        double startTime = RobotTime.getTimestampSeconds();
        mIo.readInputs(mInputs);

        logCameraInputs("Vision/CameraA", mInputs.cameraA);

        processCamera(mInputs.cameraA, "CameraA", VisionConstants.CameraA.kCameraToRobot);
    }

    private Optional<VisionFieldPoseEstimate> processCamera(
            VisionIO.VisionIOInputs.CameraInputs camInputs,
            String camName,
            Transform2d cameraToRobot) {
        String logPrefix = "Vision/" + camName;
        Optional<VisionFieldPoseEstimate> estimate = Optional.empty();

        if (!camInputs.seesTag) {
            return estimate;
        }

        if (!isOnField(camInputs.pose3d)) {
            return estimate;
        } else {
            Logger.recordOutput("Vision/CameraA/IsOnField", true);
        }

        if (camInputs.megatagPoseEstimate != null || camInputs.fiducialObservations != null) {
            Optional<VisionFieldPoseEstimate> mtEstimate =
                    processMegatagPoseEstimate(camInputs, logPrefix);

            mtEstimate.ifPresent(
                    est ->
                            Logger.recordOutput(
                                    logPrefix + "/AcceptedMegatagEstimate",
                                    est.getVisionRobotPose()));

            return mtEstimate;
        }

        // for now return optional
        return Optional.empty();
    }

    private Optional<VisionFieldPoseEstimate> processMegatagPoseEstimate(
            VisionIO.VisionIOInputs.CameraInputs camInputs, String logPrefix) {
        if (camInputs.megatagPoseEstimate.timestampSeconds()
                <= mRobotState.getLastUsedMegatagTimestamp()) {
            return Optional.empty();
        }

        // Extra checks for singular tag readings
        if (camInputs.megatagPoseEstimate.tagCount() < 2
                && camInputs.fiducialObservations[0] != null) {
            quality =
                    camInputs.megatagPoseEstimate.tagCount() > 1
                            ? 1.0
                            : 1 - camInputs.fiducialObservations[0].ambiguity();

            if (camInputs.fiducialObservations[0].ambiguity()
                    > VisionConstants.kSingleTagAmbiguityThreshold) {
                return Optional.empty();
            }

            if (camInputs.fiducialObservations[0].area()
                    < VisionConstants.kSingleTagAreaThreshold) {
                return Optional.empty();
            }

            var priorPose =
                    mRobotState.getPriorPose(camInputs.megatagPoseEstimate.timestampSeconds());
            if (priorPose.isPresent()) {
                double yawDif =
                        Math.abs(
                                MathUtil.angleModulus(
                                        priorPose.get().getRotation().getRadians()
                                                - camInputs
                                                        .megatagPoseEstimate
                                                        .fieldToRobot()
                                                        .getRotation()
                                                        .getRadians()));

                if (yawDif > Units.degreesToRadians(VisionConstants.kSingleTagYawThreshold)) {
                    return Optional.empty();
                }

                if (camInputs.megatagPoseEstimate.fieldToRobot().getTranslation().getNorm()
                        < VisionConstants.kSingleTagNormThreshold) {
                    return Optional.empty();
                }
            }
        }
        // Later would like to add local pose based on tag ID here:

        Pose2d estimatePose = camInputs.megatagPoseEstimate.fieldToRobot();
        double scaleFactor = 1 / quality;

        double xStd =
                camInputs.standardDeviations[VisionConstants.kMegatag1XStdDevIndex] * scaleFactor;
        double yStd =
                camInputs.standardDeviations[VisionConstants.kMegatag1YStdDevIndex] * scaleFactor;
        double rotStd =
                camInputs.standardDeviations[VisionConstants.kMegatag1YawStdDevIndex] * scaleFactor;

        double xyStd = Math.max(xStd, yStd);
        Matrix<N3, N1> visionStdDevs = VecBuilder.fill(xyStd, xyStd, rotStd);

        return Optional.of(
                new VisionFieldPoseEstimate(
                        estimatePose,
                        camInputs.megatagPoseEstimate.timestampSeconds(),
                        visionStdDevs,
                        camInputs.megatagPoseEstimate.tagCount()));
    }

    private void logCameraInputs(String logPrefix, VisionIO.VisionIOInputs.CameraInputs camera) {
        Logger.recordOutput(logPrefix + "/SeesTag", camera.seesTag);
        Logger.recordOutput(logPrefix + "/MegatagCount", camera.megatagCount);

        if (DriverStation.isDisabled()) {
            SmartDashboard.putBoolean(logPrefix + "/SeesTag", camera.seesTag);
            SmartDashboard.putNumber(logPrefix + "/MegatagCount", camera.megatagCount);
        }

        if (camera.pose3d != null) {
            Logger.recordOutput(logPrefix + "/Pose3d", camera.pose3d);
        }

        if (camera.megatagPoseEstimate != null) {
            Logger.recordOutput(
                    logPrefix + "/MegatagPoseEstimate", camera.megatagPoseEstimate.fieldToRobot());
            Logger.recordOutput(logPrefix + "/TagCount", camera.megatagPoseEstimate.tagCount());
            // Logger.recordOutput(logPrefix + "/AvgTagArea",
            // camera.megatagPoseEstimate.avgTagArea());
        }

        if (camera.fiducialObservations != null) {
            Logger.recordOutput(logPrefix + "/FiducialCount", camera.fiducialObservations.length);
        }
    }

    private boolean isOnField(Pose3d pose) {
        if (pose == null) {
            return false;
        }

        double poseX = pose.getX();
        double poseY = pose.getY();
        double poseZ = pose.getZ();

        Logger.recordOutput("Vision/CameraA/PoseX", poseX);
        Logger.recordOutput("Vision/CameraA/PoseY", poseY);
        Logger.recordOutput("Vision/CameraA/PoseZ", poseZ);

        return ((poseX > Constants.FieldDimensions.kMargMinX
                        && poseX < Constants.FieldDimensions.kMargMaxX)
                && (poseY > Constants.FieldDimensions.kMargMinY
                        && poseY < Constants.FieldDimensions.kMargMaxY)
                && (poseZ > Constants.FieldDimensions.kMargMinZ
                        && poseZ < Constants.FieldDimensions.kMargMaxZ));
    }
}
