package com.team236.frc2026.simulation;

import com.team236.frc2026.Constants;
import com.team236.frc2026.RobotContainer;
import com.team236.lib.robot.ConcurrentTimeInterpolatableBuffer;
import com.team236.lib.time.RobotTime;
import edu.wpi.first.math.geometry.Pose2d;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/**
 * The {@code SimulatedRobotState} tracks the simulated physical truth data of the robot over time
 * using a time-interpolatable pose buffer.
 */
public class SimulatedRobotState {

    private final ConcurrentTimeInterpolatableBuffer<Pose2d> mFieldToRobotSimulatedTruth =
            ConcurrentTimeInterpolatableBuffer.createBuffer(Constants.kLogBackTime);

    private SwerveDriveSimulation mSimDrive;
    private final RobotContainer mRobotContainer;

    public SimulatedRobotState(RobotContainer container) {
        this.mRobotContainer = container;
    }

    public void init() {
        this.mSimDrive =
                this.mRobotContainer.getDriveSubsystem().getMapleSimDrivetrain().mapleSimDrive;
    }

    public synchronized void addFieldToRobot(Pose2d pose) {
        mFieldToRobotSimulatedTruth.addSample(RobotTime.getTimestampSeconds(), pose);
    }

    public synchronized Pose2d getLatestFieldToRobot() {
        var entry = mFieldToRobotSimulatedTruth.getInternalBuffer().lastEntry();
        if (entry == null) {
            return null;
        }
        return entry.getValue();
    }

    public void updateSim() {
        // Implement simulation update logic here if/when new subsystems are added
    }

    public SwerveDriveSimulation getSimDrive() {
        return this.mSimDrive;
    }
}
