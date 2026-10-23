package com.team236.frc2026.viz;

import com.team236.frc2026.RobotState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.Logger;

public class RobotViz {
    private final RobotState state;

    private Pose3d armPivotPose3d = new Pose3d();
    private Pose3d armEndEffectorPose3d = new Pose3d();

    private static final double kArmPivotXOffset = 0.2;
    private static final double kArmPivotZOffset = 0.3;
    private static final double kArmLength = 0.5;

    public RobotViz(RobotState state) {
        this.state = state;
    }

    public void updateViz() {

        double armAngleRadians = state.getArmPivotAngleRadians();

        armPivotPose3d = new Pose3d(kArmPivotXOffset, 0, kArmPivotZOffset, new Rotation3d());

        double endEffectorX_relativeToPivot = kArmLength * Math.cos(armAngleRadians);
        double endEffectorZ_relativeToPivot = kArmLength * Math.sin(armAngleRadians);

        armEndEffectorPose3d =
                new Pose3d(
                        kArmPivotXOffset + endEffectorX_relativeToPivot,
                        0,
                        kArmPivotZOffset + endEffectorZ_relativeToPivot,
                        new Rotation3d(0, -armAngleRadians, 0));

        Logger.recordOutput("Arm/ArmPivot", armPivotPose3d);
        Logger.recordOutput("Arm/ArmEndEffector", armEndEffectorPose3d);
    }
}
