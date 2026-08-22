package com.team236.frc2026.subsystems.vision;

import com.team236.frc2026.Constants;
import com.team236.frc2026.Constants.VisionConstants;
import com.team236.frc2026.RobotState;
import com.team236.frc2026.simulation.SimulatedRobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSimPhoton extends VisionHardwareLimelight {
    private NetworkTable mTableA =
            NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightAName);
    private final PhotonCamera mCameraA = new PhotonCamera("camera");
    private final PhotonCameraSim mCameraASim;
    private final VisionSystemSim mVisionSim;
    private final SimulatedRobotState mSimRobotState;

    private final int mResWidth = 1280;
    private final int mResHeight = 800;

    public VisionSimPhoton(RobotState robotState, SimulatedRobotState simRobotState) {
        super(robotState);
        this.mSimRobotState = simRobotState;

        mVisionSim = new VisionSystemSim("main");
        mVisionSim.addAprilTags(Constants.kRebuiltAprilTagLayout);

        SimCameraProperties camAProp = new SimCameraProperties();
        camAProp.setCalibration(mResWidth, mResHeight, Rotation2d.fromDegrees(80));
        camAProp.setCalibError(0.35, 0.1);
        camAProp.setFPS(45);
        camAProp.setAvgLatencyMs(25);
        camAProp.setLatencyStdDevMs(5);
        camAProp.setExposureTimeMs(5);

        mCameraASim = new PhotonCameraSim(mCameraA, camAProp);

        mVisionSim.addCamera(mCameraASim, Constants.VisionConstants.CameraA.kCameraAToRobot);

        mCameraASim.enableRawStream(true);
        mCameraASim.enableProcessedStream(true);
        mCameraASim.enableDrawWireframe(true);
    }

    @Override
    public void readInputs(VisionIOInputs inputs) {
        Pose2d estimatedPose = mSimRobotState.getLatestFieldToRobot();
        if (estimatedPose != null) {
            mVisionSim.update(estimatedPose);
            Logger.recordOutput("Vision/SimIO/updateSimPose", estimatedPose);
        }

        writeToTable(mCameraA.getAllUnreadResults(), mTableA, mCameraASim);

        super.readInputs(inputs);
    }

    private List<Double> getBotpose(
            Transform3d fieldToCamera,
            int numTags,
            PhotonPipelineResult result,
            PhotonCameraSim cameraSim) {
        if (result == null || result.targets.isEmpty()) return null;

        Optional<Transform3d> optRobotToCamera =
                mVisionSim.getRobotToCamera(cameraSim, Timer.getFPGATimestamp());
        Pose3d fieldToRobot;
        if (optRobotToCamera.isPresent()) {
            Transform3d cameraToRobot = optRobotToCamera.get().inverse();
            Pose3d robotPose3d =
                    new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation())
                            .transformBy(cameraToRobot);
            fieldToRobot = robotPose3d;
        } else {
            fieldToRobot = new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation());
        }

        List<Double> pose_data =
                new ArrayList<>(
                        Arrays.asList(
                                fieldToRobot.getX(),
                                fieldToRobot.getY(),
                                fieldToRobot.getZ(),
                                0.0,
                                0.0,
                                fieldToRobot.getRotation().getMeasureZ().in(Units.Degree),
                                result.metadata.getLatencyMillis(),
                                (double) numTags,
                                0.0,
                                0.0,
                                result.getBestTarget().getArea()));

        for (var target : result.targets) {
            pose_data.addAll(
                    Arrays.asList(
                            (double) target.getFiducialId(),
                            target.getYaw(), // txnc
                            target.getPitch(), // tync
                            target.area, // ta
                            0.0, // distToCamera
                            0.0, // distToRobot
                            target.getPoseAmbiguity() // ambiguity
                            ));
        }
        return pose_data;
    }

    /**
     * Writes simulated vision data to NetworkTables for consumption by Limelight processing code.
     */
    private void writeToTable(
            List<PhotonPipelineResult> results, NetworkTable table, PhotonCameraSim cameraSim) {
        boolean seesTarget = false;
        for (var result : results) {
            List<Double> pose_data = null;
            if (result.getMultiTagResult().isPresent()) {
                var multiTagResult = result.getMultiTagResult().get();
                Transform3d best = multiTagResult.estimatedPose.best;

                pose_data =
                        getBotpose(best, multiTagResult.fiducialIDsUsed.size(), result, cameraSim);
            } else if (result.hasTargets()) {
                var bestTarget = result.getBestTarget();
                Transform3d best =
                        Constants.kRebuiltAprilTagLayout
                                .getTagPose(bestTarget.getFiducialId())
                                .get()
                                .minus(Pose3d.kZero)
                                .plus(bestTarget.bestCameraToTarget.inverse());

                pose_data = getBotpose(best, 1, result, cameraSim);
            }

            if (pose_data != null) {
                table.getEntry("botpose_wpiblue")
                        .setDoubleArray(
                                pose_data.stream().mapToDouble(Double::doubleValue).toArray());
                table.getEntry("botpose_orb_wpiblue")
                        .setDoubleArray(
                                pose_data.stream().mapToDouble(Double::doubleValue).toArray());
                // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll,
                // MT2pitch, MT2yaw]
                table.getEntry("stddevs")
                        .setDoubleArray(
                                new Double[] {
                                    0.3, 0.3, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                                });
                seesTarget = true;
            }
            table.getEntry("cl").setDouble(result.metadata.getLatencyMillis());
        }
        table.getEntry("tv").setInteger(seesTarget ? 1 : 0);
    }
}
