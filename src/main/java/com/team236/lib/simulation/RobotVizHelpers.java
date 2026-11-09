package com.team236.lib.simulation;

import com.team236.frc2026.Constants;
import com.team236.frc2026.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;

public class RobotVizHelpers {
    public static Optional<Pose3d> getGrabberBallFieldPose(RobotState state) {
        if (!state.getIntakePossession() || state.getLatestFieldToRobot() == null) {
            return Optional.empty();
        }

        Pose2d robotPose2d = state.getLatestFieldToRobot().getValue();
        Pose3d robotPose3d = new Pose3d(robotPose2d);

        Pose3d armPose = state.getArmPose();
        Pose3d ballRobotPose = armPose.transformBy(Constants.ArmConstants.kArmPivotToGrabber);

        Pose3d ballFieldPose =
                robotPose3d.transformBy(
                        new Transform3d(
                                ballRobotPose.getTranslation(), ballRobotPose.getRotation()));

        return Optional.of(ballFieldPose);
    }
}
