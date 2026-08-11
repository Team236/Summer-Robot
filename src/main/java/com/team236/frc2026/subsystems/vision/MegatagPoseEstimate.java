package com.team236.frc2026.subsystems.vision;

import com.team236.lib.limelight.LimelightHelpers;
import com.team236.lib.math.GeometryHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

/**
 * The {@code MegatagPoseEstimate} represents a robot pose estimate using multiple AprilTags
 * (Megatag).
 *
 * @param fieldToRobot The estimated robot pose on the field
 * @param timestampSeconds The timestamp when this estimate was captured
 * @param latency Processing latency in seconds
 * @param tagCount Number of tags used for this estimate
 * @param avgTagDist Average distance to tags in meters
 */
public record MegatagPoseEstimate(
        Pose2d fieldToRobot,
        double timestampSeconds,
        double latency,
        int tagCount,
        double avgTagDist)
        implements StructSerializable {

    public MegatagPoseEstimate {
        if (fieldToRobot == null) {
            fieldToRobot = GeometryHelpers.kPose2dZero;
        }
    }

    /** Converts a Limelight pose estimate to a MegatagPoseEstimate. */
    public static MegatagPoseEstimate fromLimelight(LimelightHelpers.PoseEstimate poseEstimate) {
        Pose2d fieldToRobot = poseEstimate.pose;
        if (fieldToRobot == null) {
            fieldToRobot = GeometryHelpers.kPose2dZero;
        }
        return new MegatagPoseEstimate(
                fieldToRobot,
                poseEstimate.timestampSeconds,
                poseEstimate.latency,
                poseEstimate.tagCount,
                poseEstimate.avgTagDist);
    }

    public static final MegatagPoseEstimateStruct struct = new MegatagPoseEstimateStruct();

    public static class MegatagPoseEstimateStruct implements Struct<MegatagPoseEstimate> {

        @Override
        public Class<MegatagPoseEstimate> getTypeClass() {
            return MegatagPoseEstimate.class;
        }

        @Override
        public String getTypeString() {
            return "record:MegatagPoseEstimate";
        }

        @Override
        public int getSize() {
            // Pose2d (72) + 3 doubles (24) + 1 int (4) + 1 double (8)
            return Pose2d.struct.getSize() + (4 * Double.BYTES) + Integer.BYTES;
        }

        @Override
        public String getSchema() {
            return "Pose2d fieldToRobot; double timestampSeconds; double latency; int tagCount; double avgTagDist";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {Pose2d.struct};
        }

        @Override
        public MegatagPoseEstimate unpack(ByteBuffer bb) {
            Pose2d fieldToRobot = Pose2d.struct.unpack(bb);
            double timestampSeconds = bb.getDouble();
            double latency = bb.getDouble();
            int tagCount = bb.getInt();
            double avgTagDist = bb.getDouble();
            return new MegatagPoseEstimate(
                    fieldToRobot, timestampSeconds, latency, tagCount, avgTagDist);
        }

        @Override
        public void pack(ByteBuffer bb, MegatagPoseEstimate value) {
            Pose2d.struct.pack(bb, value.fieldToRobot());
            bb.putDouble(value.timestampSeconds());
            bb.putDouble(value.latency());
            bb.putInt(value.tagCount());
            bb.putDouble(value.avgTagDist());
        }

        @Override
        public String getTypeName() {
            return "MegatagPoseEstimate";
        }
    }
}
