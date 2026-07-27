package com.team236.frc2026.subsystems.vision;

import com.team236.frc2026.Constants.VisionConstants;
import com.team236.frc2026.RobotState;
import com.team236.lib.limelight.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The {@code VisionHardwareLimelight} class manages the hardware inputs and network table
 * communications for the Limelight camera.
 */
public class VisionHardwareLimelight implements VisionIO {

    private NetworkTable mTableA =
            NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightAName);
    private RobotState mRobotState;
    private static int errCount;

    private static final double[] kDefaultStd = new double[VisionConstants.kStdDevArrayLength];

    public VisionHardwareLimelight(RobotState robotState) {
        this.mRobotState = robotState;
        setLLConfig();
    }

    private void setLLConfig() {
        double[] cameraAPose = {
            VisionConstants.CameraA.kRobotToCameraX,
            VisionConstants.CameraA.kRobotToCameraY,
            VisionConstants.CameraA.kCameraHeightOffGroud,
            180.0,
            VisionConstants.CameraA.kCameraPitchDegrees,
            VisionConstants.CameraA.kCameraYawOffset
        };

        mTableA.getEntry("camerapose_robotspace_set").setDoubleArray(cameraAPose);
    }

    private void readCameraData(
            NetworkTable table, VisionIOInputs.CameraInputs camInputs, String limelightName) {
        camInputs.seesTag =
                LimelightHelpers.getTV(limelightName);

        if (camInputs.seesTag) {
            try {
                var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
                var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
                var robotPose3d =
                        LimelightHelpers.toPose3D(
                                LimelightHelpers.getBotPose_wpiBlue(limelightName));

                if (megatag != null) {
                    camInputs.megatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
                    camInputs.megatagCount = megatag.tagCount;
                    camInputs.fiducialObservations =
                            FiducialObservation.fromLimelight(megatag.rawFiducials);
                }
                if (megatag2 != null) {
                    camInputs.megatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
                    camInputs.megatag2Count = megatag2.tagCount;
                }
                if (robotPose3d != null) {
                    camInputs.pose3d = robotPose3d;
                }
                camInputs.standardDeviations =
                        table.getEntry("stddevs").getDoubleArray(kDefaultStd);
            } catch (Exception err) {
                if (errCount < 1) {
                    System.err.println("Error processing vision data: " + err.getMessage());
                    errCount++;
                } else {
                    return;
                }
            }
        }
    }

    @Override
    public void readInputs(VisionIOInputs ioInputs) {
        readCameraData(mTableA, ioInputs.cameraA, VisionConstants.kLimelightAName);
    }
}
