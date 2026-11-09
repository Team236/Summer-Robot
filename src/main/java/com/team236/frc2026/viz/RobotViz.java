package com.team236.frc2026.viz;

import com.team236.frc2026.Robot;
import com.team236.frc2026.RobotState;
import com.team236.lib.simulation.RobotVizHelpers;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class RobotViz {
    private final RobotState mRobotState;

    public RobotViz(RobotState state) {
        this.mRobotState = state;
    }

    public void updateViz() {
        Logger.recordOutput("RobotViz/Arm-Pivot/ArmPose", mRobotState.getArmPose());

        if (Robot.isSimulation()) {
            Optional<Pose3d> heldBallPose = RobotVizHelpers.getGrabberBallFieldPose(mRobotState);
            if (heldBallPose.isPresent()) {
                Logger.recordOutput("RobotViz/GrabberBall", heldBallPose.get());
            } else {
                Logger.recordOutput("RobotViz/GrabberBall", new Pose3d());
            }
        }
    }
}
