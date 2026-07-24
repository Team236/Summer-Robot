package com.team236.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {

    // ** Inputs from vision system */
    public static class VisionIOInputs {

        // ** Inputs from a single camera */
        class CameraIputs {
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
        public CameraIputs cameraA = new CameraIputs();
    }

    //Interface methods that must be implemented by VisionHardware
    void readInputs(VisionIOInputs inputs);
}
