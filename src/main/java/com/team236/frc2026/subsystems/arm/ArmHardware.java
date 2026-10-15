package com.team236.frc2026.subsystems.arm;

import com.team236.frc2026.RobotState;
import com.team236.frc2026.Factories.MotorFactory;

public class ArmHardware implements ArmIO {
    private RobotState mRobotState;

    private TalonFX armMotor = MotorFactory.createArmMotor()
}
