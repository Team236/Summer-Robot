package com.team236.frc2026.subsystems.vision;

import com.team236.frc2026.RobotState;
import com.team236.lib.time.RobotTime;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private final VisionIO mIo;
  private final RobotState mRobotState;
  private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

  public VisionSubsystem(VisionIO io, RobotState robotState) {
    this.mIo = io;
    this.mRobotState = robotState;
  }

  @Override
  public void periodic() {
  }
}
