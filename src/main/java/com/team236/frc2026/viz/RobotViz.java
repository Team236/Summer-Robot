package com.team236.frc2026.viz;

import com.team236.frc2026.RobotState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.Logger;

public class RobotViz {
    private final RobotState state;

    private Pose3d mArmPose = new Pose3d();

    public RobotViz(RobotState state) {
        this.state = state;
    }

    public void updateViz() {

        double armAngleRadians = state.getArmPivotAngleRadians();

        mArmPose = new Pose3d(0, 0, 0, new Rotation3d(0, -armAngleRadians, 0));

        Logger.recordOutput("Arm-Pivot/ArmPose", mArmPose);
    }
}
