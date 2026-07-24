package com.team236.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * The {@code VisionIO} interface defines the hardware inputs and data-reading contract for the
 * vision subsystem.
 */
public interface VisionIO {

    /** The {@code VisionIOInputs} class holds all data variables from the vision system. */
    public static class VisionIOInputs {

        /** The {@code CameraInputs} class holds raw data from a single camera feed. */
        class CameraInputs {
            public boolean seesTag;
            public FiducialObservation[] fiducialObservations;
            public MegatagPoseEstimate megatagPoseEstimate;
            public MegatagPoseEstimate megatag2PoseEstimate;
            public int megatagCount;
            public int megatag2Count;
            public Pose3d pose3d;
            public double[] standardDeviations = new double[12];
        }

        // Creation of Limelight camera
        public CameraInputs cameraA = new CameraInputs();
    }

    // Interface methods that must be implemented by VisionHardware
    void readInputs(VisionIOInputs inputs);
}
