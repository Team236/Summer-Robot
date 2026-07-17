// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.team236.frc2026;

import com.team236.frc2026.subsystems.drive.DrivetrainProfile;
import com.team236.frc2026.subsystems.drive.PracTunerConstants;
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

    public static final class Controller {
        public static final byte kMainController = 0;
    }

    public static final class DriveConstants {
        public static final double kOpenLoopDeadband = 0.05;

        public static final double kMaxDriveSpeed = 3.0;
        public static final double kMaxRotationalRate = 3.0;

        public static final DrivetrainProfile kDrivetrain =
                PracTunerConstants.createDrivetrain(); // Add logic later for comp + prac + sim
    }
}
