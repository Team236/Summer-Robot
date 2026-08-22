package com.team236.frc2026;

import com.team236.frc2026.subsystems.drive.CompTunerConstants;
import com.team236.frc2026.subsystems.drive.DrivetrainProfile;
import com.team236.frc2026.subsystems.drive.PracTunerConstants;
import com.team236.lib.robot.NetworkHelpers;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The {@code Constants} defines the runtime mode and globally accessible hardware, tuning, and
 * structural values for the robot.
 */
public final class Constants {

    public static final Mode kSimMode = Mode.SIM;
    public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : kSimMode;

    public enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final boolean kIsConnectedViaWifi = true;
    public static final boolean kUseMapleSim = true;
    public static final boolean kUseMapleSimFuel = false;

    private static final String kPracticeBotMacAddress = "00:80:2F:40:FC:4A";
    public static boolean kIsPracticeBot = NetworkHelpers.hasMacAddress(kPracticeBotMacAddress);

    public static final double kLogBackTime = 1.0;

    /** The {@code Controller} holds constants for driver input devices. */
    public static final class Controller {
        public static final byte kMainController = 0;
    }

    /** The {@code TestbedConstants} contains physical properties of the testbed. */
    public static final class TestbedConstants {
        public static final double kRobotWeightPounds = 50.0;
        public static final double kBumperLengthInches = 36.0;
        public static final double kBumperWidthInches = 36.0;
        public static final double kBumperHeightInches = 5.0;
        public static final double kWheelCoefficientOfFriction = 1.2;
        public static final int kDriveMotorCount = 1;
    }

    /** The {@code DriveConstants} holds tuning and configuration values for the drivetrain. */
    public static final class DriveConstants {
        public static final double kOpenLoopDeadband = 0.05;

        public static final double kMaxDriveSpeed = 3.0;
        public static final double kMaxRotationalRate = 3.0;

        public static final DrivetrainProfile kDrivetrain =
                kIsPracticeBot
                        ? PracTunerConstants.createDrivetrain()
                        : CompTunerConstants.createDrivetrain();
    }

    /** The {@code VisionConstants} defines camera and AprilTag pipeline configurations. */
    public static final class VisionConstants {
        public static final String kLimelightAName = "limelight";

        public static final int kStdDevArrayLength = 12;
        public static final int kMegatag1XStdDevIndex = 0;
        public static final int kMegatag1YStdDevIndex = 1;
        public static final int kMegatag1YawStdDevIndex = 5;

        public static final int kMegatag2XStdDevIndex = 6;
        public static final int kMegatag2YStdDevIndex = 7;
        public static final int kMegatag2YawStdDevIndex = 11;

        public static final double kSingleTagAmbiguityThreshold = 1 - 0.19;
        public static final double kSingleTagAreaThreshold = 0.5;
        public static final double kSingleTagYawThreshold = 5.0;
        public static final double kSingleTagNormThreshold = 0.5;

        /** The {@code CameraA} holds specific transform data for the primary Limelight. */
        public static final class CameraA {
            // Inches
            public static final double kRobotToCameraX = 0.0;
            public static final double kRobotToCameraY = 0.0;
            public static final double kCameraHeightOffGroud = 0.0;
            public static final double kCameraPitchDegrees = 0.0;
            public static final double kCameraYawOffset = 0.0;

            public static final Transform2d kCameraToRobot =
                    new Transform2d(
                            new Translation2d(
                                    Units.inchesToMeters(kRobotToCameraX),
                                    Units.inchesToMeters(kRobotToCameraY)),
                            new Rotation2d(0.0));
        }
    }

    /** The {@code FieldDimensions} defines the spatial bounding box of the game field. */
    public static final class FieldDimensions {
        private static final double kFieldMinX = 0.0;
        private static final double kFieldMaxX = 16.540988;
        private static final double kFieldMinY = 0.0;
        private static final double kFieldMaxY = 8.069326;
        private static final double kFieldMinZ = 0.0;
        private static final double kFieldMaxZ = 1.0;

        private static final double kFieldBoarderMargin = 0.254;

        public static final double kMargMinX = kFieldMinX - kFieldBoarderMargin;
        public static final double kMargMaxX = kFieldMaxX + kFieldBoarderMargin;
        public static final double kMargMinY = kFieldMinY - kFieldBoarderMargin;
        public static final double kMargMaxY = kFieldMaxY + kFieldBoarderMargin;
        public static final double kMargMinZ = kFieldMinZ - kFieldBoarderMargin;
        public static final double kMargMaxZ = kFieldMaxZ;
    }
}
