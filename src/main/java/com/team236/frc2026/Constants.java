// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.team236.frc2026;

import com.team236.frc2026.subsystems.drive.CompTunerConstants;
import com.team236.frc2026.subsystems.drive.DrivetrainProfile;
import com.team236.frc2026.subsystems.drive.PracTunerConstants;
import com.team236.lib.robot.NetworkHelpers;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final boolean kIsConnectedViaWifi = true;

    public static final String kPracticeBotMacAddress = "00:80:2F:40:FC:4A";
    public static boolean kIsPracticeBot = NetworkHelpers.hasMacAddress(kPracticeBotMacAddress);

    public static final class Controller {
        public static final byte kMainController = 0;
    }

    public static final class DriveConstants {
        public static final double kOpenLoopDeadband = 0.05;

        public static final double kMaxDriveSpeed = 3.0;
        public static final double kMaxRotationalRate = 3.0;

        public static final DrivetrainProfile kDrivetrain =
                kIsPracticeBot
                        ? PracTunerConstants.createDrivetrain()
                        : CompTunerConstants
                                .createDrivetrain(); // Add logic later for comp + prac + sim
    }

    public static final class VisionConstants {
        public static final String kLimelightAName = "limelight";
        public static final int kStdDevArrayLength = 12;

        public static final class CameraA {
            public static final double kRobotToCameraX = 0.0;
            public static final double kRobotToCameraY = 0.0;
            public static final double kCameraHeightOffGroud = 0.0;
            public static final double kCameraPitchDegrees = 0.0;
            public static final double kCameraYawOffset = 0.0;
        }
    }
    
    public static final class FieldDimentions {
            public static final double kFieldMinX = 0.0;
            public static final double kFieldMaxX = 16.540988;
            public static final double kFieldMinY = 0.0;
            public static final double kFieldMaxY = 8.069326;
            public static final double kFieldMinZ = 0.0;
            public static final double kFieldMaxZ = 1.0;
    }
}
