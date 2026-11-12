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
            Optional<Pose3d> fuelPose = RobotVizHelpers.getGrabberBallFieldPose(mRobotState);
            if (!fuelPose.isEmpty()) {
                Logger.recordOutput("RobotViz/GrabberBall", fuelPose.get());
            } else {
                Logger.recordOutput("RobotViz/GrabberBall", new Pose3d());
            }
        }
    }
}
