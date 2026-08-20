package com.team236.frc2026.simulation;

import com.team236.frc2026.Constants;
import com.team236.frc2026.RobotContainer;
import com.team236.frc2026.RobotState;
import com.team236.lib.time.RobotTime;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class SimulatedRobotState {
    private TimeInterpolatableBuffer<Pose2d> fieldToRobotSimulatedTruth =
            TimeInterpolatableBuffer.createBuffer(1.0); // RobotState.LOOKBACK_TIME might not exist yet, defaulting to 1.0s

    private SwerveDriveSimulation simDrive;
    private final RobotContainer container;

    public SimulatedRobotState(RobotContainer container) {
        this.container = container;
    }

    // Do this after construction to avoid circular dependencies.
    public void init() {
        if (Constants.useMapleSim && this.container.getDriveSubsystem().getMapleSimDrive() != null) {
            this.simDrive = this.container.getDriveSubsystem().getMapleSimDrive().mapleSimDrive;
        }
    }

    public synchronized void addFieldToRobot(Pose2d pose) {
        fieldToRobotSimulatedTruth.addSample(RobotTime.getTimestampSeconds(), pose);
    }

    public synchronized Pose2d getLatestFieldToRobot() {
        var entry = fieldToRobotSimulatedTruth.getInternalBuffer().lastEntry();
        if (entry == null) {
            return null;
        }
        return entry.getValue();
    }

    public void updateSim() {
        // Implement simulation update logic here if/when new subsystems are added
    }
}
