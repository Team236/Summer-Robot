package com.team236.frc2026.subsystems.vision;

import com.team236.frc2026.RobotState;
import com.team236.lib.time.RobotTime;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
    private final VisionIO mIo;
    private final RobotState mRobotState;
    private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    public VisionSubsystem(VisionIO io, RobotState robotState) {
        this.mIo = io;
        this.mRobotState = robotState;
    }

    @Override
    public void periodic() {
        double startTime = RobotTime.getTimestampSeconds();
        mIo.readInputs(inputs);

        logCameraInputs("Vision/CameraA", inputs.cameraA);
    }

    private void logCameraInputs(String prefix, VisionIO.VisionIOInputs.CameraInputs camera) {
        Logger.recordOutput(prefix + "/SeesTag", camera.seesTag);
        Logger.recordOutput(prefix + "/MegatagCount", camera.megatagCount);

        if (DriverStation.isDisabled()) {
            SmartDashboard.putBoolean(prefix + "/SeesTag", camera.seesTag);
            SmartDashboard.putNumber(prefix + "/MegatagCount", camera.megatagCount);
        }

        if (camera.pose3d != null) {
            Logger.recordOutput(prefix + "/Pose3d", camera.pose3d);
        }

        if (camera.megatagPoseEstimate != null) {
            Logger.recordOutput(
                    prefix + "/MegatagPoseEstimate", camera.megatagPoseEstimate.fieldToRobot());
            Logger.recordOutput(prefix + "/Quality", camera.megatagPoseEstimate.quality());
            Logger.recordOutput(prefix + "/AvgTagArea", camera.megatagPoseEstimate.avgTagArea());
        }

        if (camera.fiducialObservations != null) {
            Logger.recordOutput(prefix + "/FiducialCount", camera.fiducialObservations.length);
        }
    }
}
