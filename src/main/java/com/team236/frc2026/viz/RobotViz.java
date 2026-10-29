package com.team236.frc2026.viz;

import com.team236.frc2026.Constants;
import com.team236.frc2026.RobotState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;

public class RobotViz {
    private final RobotState mRobotState;
    private final Translation3d mArmPivotPoint = Constants.ArmConstants.kArmSimPivotPoint;

    private Pose3d mArmPose = new Pose3d();

    public RobotViz(RobotState state) {
        this.mRobotState = state;
    }

    public void updateViz() {
        double armAngleRadians = mRobotState.getArmPivotAngleRadians();

        // Rotate around the pivot point rather than the CAD origin
        Rotation3d rotation = new Rotation3d(0, armAngleRadians, 0);
        mArmPose =
                new Pose3d(mArmPivotPoint, rotation)
                        .transformBy(
                                new Transform3d(mArmPivotPoint.unaryMinus(), new Rotation3d()));

        Logger.recordOutput("Arm-Pivot/ArmPose", mArmPose);
        Logger.recordOutput("Arm-Pivot/ArmEndEffector", mArmPose);
    }
}
