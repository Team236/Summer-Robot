// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.team236.frc2026;

import com.ctre.phoenix6.SignalLogger;
import com.team236.lib.limelight.Limelight3GConfig;
import com.team236.lib.simulation.FuelPhysicsSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private RobotContainer mRobotContainer;
    private FuelPhysicsSim mBallSim;

    public Robot() {
        SignalLogger.enableAutoLogging(false);

        // Testbed configuration
        if (Constants.kIsPracticeBot) Limelight3GConfig.configureNTLimelight3G();

        // Record robot code metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
                "GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // Start AdvantageKit logger
        Logger.start();

        mRobotContainer = new RobotContainer();

        if (RobotBase.isSimulation()) {
            mRobotContainer.getDriveSubsystem().resetOdometry(new Pose2d(3, 3, new Rotation2d()));
        }
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if (Robot.isSimulation()) {
            mRobotContainer.getSimulatedRobotState().updateSim();
        }
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {}

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {}

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {}

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
        mBallSim = new FuelPhysicsSim("Sim/Fuel");
        mBallSim.enable();
        mBallSim.placeFieldBalls();

        // tell it about your robot
        mBallSim.configureRobot(
                Units.inchesToMeters(Constants.SimulationConstants.kBumperWidthInches),
                Units.inchesToMeters(Constants.SimulationConstants.kBumperLengthInches),
                Units.inchesToMeters(10), // bumper height
                () -> {
                    Pose2d pose = mRobotContainer.getSimulatedRobotState().getLatestFieldToRobot();
                    return pose != null ? pose : new Pose2d();
                },
                () -> {
                    var simDrive = mRobotContainer.getDriveSubsystem().getMapleSimDrive();
                    if (simDrive != null) {
                        return simDrive.mapleSimDrive
                                .getDriveTrainSimulatedChassisSpeedsRobotRelative();
                    }
                    return new ChassisSpeeds();
                });
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        if (mBallSim != null) {
            mBallSim.tick();
        }
    }
}
