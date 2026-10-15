package com.team236.frc2026.Factories;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MotorFactory {

    public static TalonFX createArmMotor (int id, String network) {
        TalonFX armMotor = new TalonFX(id, network);
        TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();

        armMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        armMotorConfig.CurrentLimits.SupplyCurrentLimit = 0; // Needs Update
        armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var slot0Config = armMotorConfig.Slot0;  
        slot0Config.kV = 0;
        slot0Config.kP = 0;
        slot0Config.kI = 0;
        slot0Config.kD = 0;

        armMotor.getConfigurator().apply(armMotorConfig);

        return armMotor;
    }
}
