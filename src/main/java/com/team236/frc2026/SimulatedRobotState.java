package com.team236.frc2026;

import com.team236.lib.time.RobotTime;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/**
 * The {@code SimulatedRobotState} tracks the simulated physical truth data 
 * of the robot over time using a time-interpolatable pose buffer.
 */
public class SimulatedRobotState {

    private final TimeInterpolatableBuffer<Pose2d> mFieldToRobotSimulatedTruth =
            TimeInterpolatableBuffer.createBuffer(RobotState.kLogBackTime);
    
    private SwerveDriveSimulation mSimDrive;
    private final RobotContainer mContainer;

    public SimulatedRobotState(RobotContainer container) {
        this.mContainer = container;
    }

    public void init() {
        this.mSimDrive = this.mContainer.getDriveSubsystem().getMapleSimDrive().mapleSimDrive;
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
}