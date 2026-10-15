package com.team236.frc2026.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team236.frc2026.Constants;
import com.team236.frc2026.Factories.MotorFactory;
import com.team236.frc2026.RobotState;

public class ArmHardware implements ArmIO {
    private RobotState mRobotState;

    private TalonFX armMotor =
            MotorFactory.createArmMotor(
                    Constants.ArmConstants.kMotorId, Constants.ArmConstants.kMotorNetwork);

    public void readInputs(ArmIOInputs inputs) {}

    public void setPosition(double pose) {}

    public void setVoltage(double voltage) {}

    public void resetEncoder() {}
}
