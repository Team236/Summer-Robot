package com.team236.frc2026.factories;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team236.frc2026.Constants;

public class MotorFactory {

    public static TalonFX createArmMotor(int id, String network) {
        TalonFX armMotor = new TalonFX(id, network);
        TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();

        armMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        armMotorConfig.CurrentLimits.SupplyCurrentLimit =
                Constants.ArmConstants.kSupplyCurrentLimit; // Needs Update
        armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var slot0Config = armMotorConfig.Slot0;
        slot0Config.kV = Constants.ArmConstants.kSlot0Kv;
        slot0Config.kP = Constants.ArmConstants.kSlot0Kp;
        slot0Config.kI = Constants.ArmConstants.kSlot0Ki;
        slot0Config.kD = Constants.ArmConstants.kSlot0Kd;

        armMotor.getConfigurator().apply(armMotorConfig);

        return armMotor;
    }
}
