package com.team236.frc2026;

import com.team236.frc2026.subsystems.drive.CompTunerConstants;
import com.team236.frc2026.subsystems.drive.DrivetrainProfile;
import com.team236.frc2026.subsystems.drive.PracTunerConstants;
import com.team236.lib.roborio.NetworkHelpers;
import com.team236.lib.subsystems.cancoder.CanDeviceId;
import com.team236.lib.subsystems.motor.MotorSubsystemWithCanCoderConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
    public static final boolean kUseMapleSim = false;
    public static final boolean kUseMapleSimFuel = false;

    private static final String kPracticeBotMacAddress = "00:80:2F:40:FC:4A";
    public static boolean kIsPracticeBot = NetworkHelpers.hasMacAddress(kPracticeBotMacAddress);

    public static final double kLogBackTime = 1.0;

    public static final AprilTagFieldLayout kRebuiltAprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static final String kCanBusDrivebase = "drivebase";

    public static final double kEpsilonMotorTolerance = 0.005;

    /** The {@code Controller} holds constants for driver input devices. */
    public static final class Controller {
        public static final byte kMainController = 0;
        public static final byte kOperatorController = 1;
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
            public static final double kCameraHeightOffGroud = 6.0;
            public static final double kCameraPitchDegrees = 0.0;
            public static final double kCameraRoll = 0.0;
            public static final double kCameraPitch = -30.0;
            public static final double kCameraYawOffset = 0.0;

            public static final Transform3d kRobotToCameraA =
                    new Transform3d(
                            new Translation3d(
                                    Units.inchesToMeters(kRobotToCameraX),
                                    Units.inchesToMeters(kRobotToCameraY),
                                    Units.inchesToMeters(kCameraHeightOffGroud)),
                            new Rotation3d(
                                    Units.degreesToRadians(kCameraRoll),
                                    Units.degreesToRadians(kCameraPitch),
                                    Units.degreesToRadians(kCameraYawOffset)));
        }
    }

    /** The {@code FieldDimensions} defines the arm subsystem. */
    public static final class ArmConstants {
        public static final int kARmPivotMotorId = 1;
        public static final MotorSubsystemWithCanCoderConfig kArmPivotConfig =
                new MotorSubsystemWithCanCoderConfig();

        static {
            kArmPivotConfig.canCoderConfig.canId =
                    new CanDeviceId(Constants.ArmConstants.kARmPivotMotorId, kCanBusDrivebase);
        }

        // Motor Configs (In static):
        public static final String kMotorNetwork = "rio";
        public static final double kSupplyCurrentLimit = 35.0;
        public static final double kSlot0Kv = 0.0;
        public static final double kSlot0Kp = 0.0;
        public static final double kSlot0Ki = 0.0;
        public static final double kSlot0Kd = 0.0;

        public static final double kArmPivotStowPositionUnits = 0.0;
        public static final double kArmPivotIntakePositionUnits = 1.0;

        public static final double kArmPivotRetractRadians = 0.0;
        public static final double kArmPivotExtendRadians = 3.14;

        // Manual open-loop duty cycle for arm pivot (tune this value).
        public static final double kArmPivotManualDutyCycle = 0.2;
    }

    /** The {@code FieldDimensions} defines the spatial bounding box of the game field. */
    public static final class FieldDimensions {
        public static final double kFieldLengthMeters = kRebuiltAprilTagLayout.getFieldLength();
        public static final double kFieldWidthMeters = kRebuiltAprilTagLayout.getFieldWidth();
        public static final double kFieldHeightMeters = 0;

        public static final double kFieldMinX = 0.0;
        public static final double kFieldMinY = 0.0;
        public static final double kFieldMinZ = 0.0;

        private static final double kFieldBoarderMargin = 0.254;

        public static final double kMargMinX = kFieldMinX - kFieldBoarderMargin;
        public static final double kMargMaxX = kFieldLengthMeters + kFieldBoarderMargin;
        public static final double kMargMinY = kFieldMinY - kFieldBoarderMargin;
        public static final double kMargMaxY = kFieldWidthMeters + kFieldBoarderMargin;
        public static final double kMargMinZ = kFieldMinZ - kFieldBoarderMargin;
        public static final double kMargMaxZ = kFieldHeightMeters + kFieldBoarderMargin;
    }
}
