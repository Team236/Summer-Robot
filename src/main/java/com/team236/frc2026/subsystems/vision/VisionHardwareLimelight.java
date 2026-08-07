package com.team236.frc2026.subsystems.vision;

import com.team236.frc2026.Constants.VisionConstants;
import com.team236.frc2026.RobotState;
import com.team236.lib.limelight.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The {@code VisionHardwareLimelight} class manages the hardware inputs and network table
 * communications for the Limelight camera.
 */
public class VisionHardwareLimelight implements VisionIO {

    private NetworkTable mTableA =
            NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightAName);
    private static final double[] kDefaultStd = new double[VisionConstants.kStdDevArrayLength];
    private RobotState mRobotState;
    private static int mErrCount;

    public VisionHardwareLimelight(RobotState robotState) {
        this.mRobotState = robotState;
        setLLConfig();
    }

    @Override
    public void readInputs(VisionIOInputs ioInputs) {
        readCameraData(mTableA, ioInputs.cameraA, VisionConstants.kLimelightAName);
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
        camInputs.seesTag = LimelightHelpers.getTV(limelightName);

        if (camInputs.seesTag) {
            updatePoseEstimates(table, camInputs, limelightName);
        }
    }

    private void updatePoseEstimates(
            NetworkTable table, VisionIOInputs.CameraInputs camInputs, String limelightName) {
        try {
            var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
            var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

            if (megatag != null) {
                camInputs.megatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
                camInputs.megatagCount = megatag.tagCount;
                camInputs.pose3d = new Pose3d(megatag.pose);
            }
            if (megatag2 != null) {
                camInputs.megatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
                camInputs.megatag2Count = megatag2.tagCount;
            }

            camInputs.standardDeviations = table.getEntry("stddevs").getDoubleArray(kDefaultStd);
        } catch (Exception err) {
            System.err.printf(
                    "Error # %d reading Limelight data: %s%n", mErrCount, err.getMessage());
            mErrCount++;
        }
    }
}
