package com.team236.frc2026.viz;

import com.team236.frc2026.RobotState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class RobotViz {
    private final RobotState state;

    // Pivot point relative to the CAD origin (Forward = +X, Right = -Y, Up = +Z)
    private static final Translation3d kPivotPoint =
            new Translation3d(
                    Units.inchesToMeters(14.268376),
                    -Units.inchesToMeters(3.390625),
                    Units.inchesToMeters(8.7));

    private Pose3d mArmPose = new Pose3d();

    public RobotViz(RobotState state) {
        this.state = state;
    }

    public void updateViz() {
        double armAngleRadians = state.getArmPivotAngleRadians();

        // Rotate around the pivot point rather than the CAD origin
        Rotation3d rotation = new Rotation3d(0, -armAngleRadians, 0);
        mArmPose =
                new Pose3d(kPivotPoint, rotation)
                        .transformBy(new Transform3d(kPivotPoint.unaryMinus(), new Rotation3d()));

        Logger.recordOutput("Arm-Pivot/ArmPose", mArmPose);
        Logger.recordOutput("Arm-Pivot/ArmEndEffector", mArmPose);
    }
}
