package com.team236.frc2026.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

    @AutoLog
    public class ArmIOInputs {
        public Pose3d pose;
        public Rotation2d rotation;
        public double velocityRadPerSec;
        public double encoderRevs;
        public double motorTemp;
        public double motorAmps;
        public boolean forwardLimitHit;
        public boolean backwardLimitHit;
    }

    public void readInputs(ArmIOInputs inputs);

    public void setPosition(double pose);

    public void setVoltage(double voltage);

    public void resetEncoder();
}
