package com.team236.frc2026.subsystems.arm;

import com.team236.frc2026.RobotState;
import com.team236.lib.subsystems.cancoder.CanCoderIO;
import com.team236.lib.subsystems.cancoder.CanCoderInputsAutoLogged;
import com.team236.lib.subsystems.motor.MotorIO;
import com.team236.lib.subsystems.motor.MotorInputsAutoLogged;
import com.team236.lib.subsystems.motor.MotorSubsystemWithCanCoder;
import com.team236.lib.subsystems.motor.MotorSubsystemWithCanCoderConfig;

/**
 * The {@code ArmIntakeSubsystem} controls the intake rollers of the robot's arm. It manages the
 * foward and reverse spinning of the arm rollers using a motor with an integrated CANcoder for
 * precise position control and feedback.
 */
public class ArmIntakeSubsystem
        extends MotorSubsystemWithCanCoder<
                MotorInputsAutoLogged, MotorIO, CanCoderInputsAutoLogged, CanCoderIO> {
    private RobotState mRobotState;

    public ArmIntakeSubsystem(
            final MotorSubsystemWithCanCoderConfig motorConfig,
            final MotorIO motorIO,
            final CanCoderIO canCoderIO,
            RobotState state) {
        super(
                motorConfig,
                new MotorInputsAutoLogged(),
                motorIO,
                new CanCoderInputsAutoLogged(),
                canCoderIO);

        this.mRobotState = state;
    }
}
