package com.team236.frc2026.subsystems.vision;

import com.team236.frc2026.Constants.VisionConstants;
import com.team236.frc2026.RobotState;
import com.team236.lib.limelight.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.concurrent.atomic.AtomicReference;

public class VisionHardwareLimelight implements VisionIO {
    NetworkTable tableA =
            NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightAName);
    RobotState robotState;
    AtomicReference<VisionIOInputs> visionCache = new AtomicReference<>(new VisionIOInputs());
    int imuMode = 1;

    private static final double[] kDefaultStd = new double[VisionConstants.kStdDevArrayLength];

    void VisionHardwareLimelight(RobotState robotState) {
        this.robotState = robotState;
        setLLConfig();
    }

    private void setLLConfig() {
        double[] cameraAPose = {
            VisionConstants.CameraA.kRobotToCameraX,
            VisionConstants.CameraA.kRobotToCameraY,
            VisionConstants.CameraA.kCameraHeightOffGroud,
            0.0,
            VisionConstants.CameraA.kCameraPitchDegrees,
            VisionConstants.CameraA.kCameraYawOffset
        };

        tableA.getEntry("camerapose_robotspace_set").setDoubleArray(cameraAPose);
    }

    private void readCameraData(
            NetworkTable table, VisionIOInputs.CameraIputs camInputs, String limelightName) {
        camInputs.seesTag = table.getEntry("tv").getDouble(0) == 1;

        if (camInputs.seesTag) {
            try {
                var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
                var robotPose3d =
                        LimelightHelpers.toPose3D(
                                LimelightHelpers.getBotPose_wpiBlue(limelightName));

                if (megatag != null) {
                    camInputs.megatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
                    camInputs.megatagCount = megatag.tagCount;
                    camInputs.fiducialObservations =
                            FiducialObservation.fromLimelight(megatag.rawFiducials);
                }
                if (robotPose3d != null) {
                    camInputs.pose3d = robotPose3d;
                }
                camInputs.standardDeviations =
                        table.getEntry("stddevs").getDoubleArray(kDefaultStd);
            } catch (Exception err) {
                System.err.println("Error processing vision data: " + err.getMessage());
            }
        }
    }

    @Override
    public void readInputs(VisionIOInputs inputs) {
        readCameraData(tableA, inputs.cameraA, VisionConstants.kLimelightAName);

        visionCache.set(inputs);
    }
}
