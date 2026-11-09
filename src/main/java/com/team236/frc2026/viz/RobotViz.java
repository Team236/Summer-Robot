package com.team236.frc2026.viz;

import com.team236.frc2026.RobotState;
import org.littletonrobotics.junction.Logger;

public class RobotViz {
    private final RobotState mRobotState;

    public RobotViz(RobotState state) {
        this.mRobotState = state;
    }

    public void updateViz() {
        Logger.recordOutput("RobotViz/Arm-Pivot/ArmPose", mRobotState.getArmPose());
    }
}
