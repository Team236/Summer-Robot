package com.team236.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The {@code GeometryHelpers} provides common static constants for robot geometry and mathematical
 * operations.
 */
public class GeometryHelpers {

    public static final Pose2d kPose2dZero = new Pose2d();

    public static final Rotation2d kRotation2dZero = new Rotation2d();
    public static final Rotation2d kRotation2dPi = Rotation2d.fromDegrees(180.0);
}
