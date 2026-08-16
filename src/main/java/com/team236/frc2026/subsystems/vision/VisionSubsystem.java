package com.team236.frc2026.subsystems.vision;

import com.team236.frc2026.Constants;
import com.team236.frc2026.Constants.VisionConstants;
import com.team236.frc2026.RobotState;
import com.team236.lib.time.RobotTime;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
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

    public VisionSubsystem(VisionIO io, RobotState robotState) {
        this.mIo = io;
        this.mRobotState = robotState;
    }

    @Override
    public void periodic() {
        double startTime = RobotTime.getTimestampSeconds();
        mIo.readInputs(mInputs);

        logCameraInputs("Vision/CameraA", mInputs.cameraA);

        Optional<VisionFieldPoseEstimate> maybeMTA = processCamera(mInputs.cameraA, "CameraA", VisionConstants.CameraA.kCameraToRobot);

        maybeMTA.ifPresent(est -> 
            mRobotState.updateMegatagEstimate(est));
    }

    private Optional<VisionFieldPoseEstimate> processCamera(
            VisionIO.VisionIOInputs.CameraInputs camInputs,
            String camName,
            Transform2d cameraToRobot) {
        String logPrefix = "Vision/" + camName;

        if (!camInputs.seesTag) {
            return Optional.empty();
        }

        Optional<VisionFieldPoseEstimate> mt1Estimate = Optional.empty();
        Optional<VisionFieldPoseEstimate> mt2Estimate = Optional.empty();
        
        if (camInputs.megatagPoseEstimate != null && camInputs.fiducialObservations != null) {
            mt1Estimate = processMegatagPoseEstimate(camInputs, logPrefix);

            mt1Estimate.ifPresent(
                    est ->
                            Logger.recordOutput(
                                    logPrefix + "/AcceptedMegatag1Pose",
                                    est.getVisionRobotPose())
                        );
        }

        if (camInputs.megatag2PoseEstimate != null && camInputs.fiducialObservations != null) {
            mt2Estimate = processMegatag2PoseEstimate(camInputs, logPrefix);

            mt2Estimate.ifPresent(
                    est ->
                            Logger.recordOutput(
                                    logPrefix + "/AcceptedMegatag2Pose",
                                    est.getVisionRobotPose()));
        }

        if(camInputs.megatagCount < 2 && mt2Estimate.isPresent()) {
            Logger.recordOutput(logPrefix + "/AcceptedMegatag1Estimate", false);
            Logger.recordOutput(logPrefix + "/AcceptedMegatag2Estimate", true);
            return mt2Estimate;
        } else if (mt1Estimate.isPresent()) {
            Logger.recordOutput(logPrefix + "/AcceptedMegatag1Estimate", true);
            Logger.recordOutput(logPrefix + "/AcceptedMegatag2Estimate", false);
            return mt1Estimate;
        } else {
            Logger.recordOutput(logPrefix + "/AcceptedMegatag1Estimate", false);
            Logger.recordOutput(logPrefix + "/AcceptedMegatag2Estimate", false);
            return Optional.empty();
        }
    }

    private Optional<VisionFieldPoseEstimate> processMegatag2PoseEstimate(
            VisionIO.VisionIOInputs.CameraInputs camInputs, String logPrefix) {

        if (camInputs.megatag2PoseEstimate.tagCount() == 0) {
            return Optional.empty();
        }

        if (camInputs.megatag2PoseEstimate.timestampSeconds()
                <= mRobotState.getLastUsedMegatagTimestamp()) {
            return Optional.empty();
        }

        Pose2d mt2Estimate = camInputs.megatag2PoseEstimate.fieldToRobot();

        if (mt2Estimate == null || !isOnField(mt2Estimate)) {
            return Optional.empty();
        }

        if (camInputs.megatag2PoseEstimate.fieldToRobot().getTranslation().getNorm()
                < VisionConstants.kSingleTagNormThreshold) {
            return Optional.empty();
        }

        double revArea = 1.0;

        for(FiducialObservation obs : camInputs.fiducialObservations) {
            revArea -= obs.area();
        }

        double scaleFactor = 1.0 / revArea;
        double xStd = camInputs.standardDeviations[VisionConstants.kMegatag2XStdDevIndex] * scaleFactor;
        double yStd = camInputs.standardDeviations[VisionConstants.kMegatag2YStdDevIndex] * scaleFactor;
        double rotStd = camInputs.standardDeviations[VisionConstants.kMegatag2YawStdDevIndex] * scaleFactor;

        double xyStd = Math.max(xStd, yStd);
        Matrix<N3, N1> visionStdDevs = VecBuilder.fill(xyStd, xyStd, rotStd);

        return Optional.of(
                new VisionFieldPoseEstimate(
                        mt2Estimate,
                        camInputs.megatag2PoseEstimate.timestampSeconds(),
                        visionStdDevs,
                        camInputs.megatag2PoseEstimate.tagCount()));
    }

    private Optional<VisionFieldPoseEstimate> processMegatagPoseEstimate(
            VisionIO.VisionIOInputs.CameraInputs camInputs, String logPrefix) {

        if (camInputs.megatagPoseEstimate.tagCount() == 0) {
            return Optional.empty();
        }

        if (camInputs.megatagPoseEstimate.timestampSeconds()
                <= mRobotState.getLastUsedMegatagTimestamp()) {
            return Optional.empty();
        }

        Pose2d mt1Estimate = camInputs.megatagPoseEstimate.fieldToRobot();

        if (mt1Estimate == null || !isOnField(mt1Estimate)) {
            return Optional.empty();
        }

        double quality = 1.0;

        // Extra checks for singular tag readings
        if (camInputs.megatagPoseEstimate.tagCount() < 2
                && camInputs.fiducialObservations[0] != null) {
            quality = 1.0 - camInputs.fiducialObservations[0].ambiguity();

            if (quality < VisionConstants.kSingleTagAmbiguityThreshold) {
                return Optional.empty();
            }

            if (camInputs.fiducialObservations[0].area()
                    < VisionConstants.kSingleTagAreaThreshold) {
                return Optional.empty();
            }

            if (camInputs.megatagPoseEstimate.fieldToRobot().getTranslation().getNorm()
                        < VisionConstants.kSingleTagNormThreshold) {
                    return Optional.empty();
                }

            var priorPose =
                    mRobotState.getPriorPose(camInputs.megatagPoseEstimate.timestampSeconds());
            if (priorPose.isPresent()) {
                double yawDif =
                        Math.abs(
                                MathUtil.angleModulus(
                                        priorPose.get().getRotation().getRadians()
                                                - mt1Estimate.getRotation().getRadians()));

                if (yawDif > Units.degreesToRadians(VisionConstants.kSingleTagYawThreshold)) {
                    return Optional.empty();
                }
            }
        }

        if(quality < 0.01) {
            quality = 0.01;
        }

        double scaleFactor = 1.0 / quality;

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
                        mt1Estimate,
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
        }

        if (camera.fiducialObservations != null) {
            Logger.recordOutput(logPrefix + "/FiducialCount", camera.fiducialObservations.length);
        }
    }

    private boolean isOnField(Pose2d pose) {
        if (pose == null) {
            return false;
        }

        double poseX = pose.getX();
        double poseY = pose.getY();

        return ((poseX > Constants.FieldDimensions.kMargMinX
                        && poseX < Constants.FieldDimensions.kMargMaxX)
                && (poseY > Constants.FieldDimensions.kMargMinY
                        && poseY < Constants.FieldDimensions.kMargMaxY));
    }
}
