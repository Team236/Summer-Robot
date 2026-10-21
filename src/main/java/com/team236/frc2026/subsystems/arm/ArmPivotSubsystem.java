package com.team236.frc2026.subsystems.arm;

import com.team236.frc2026.Constants;
import com.team236.frc2026.RobotState;
import com.team236.lib.math.MathHelpers;
import com.team236.lib.subsystems.cancoder.CanCoderIO;
import com.team236.lib.subsystems.cancoder.CanCoderInputsAutoLogged;
import com.team236.lib.subsystems.motor.MotorIO;
import com.team236.lib.subsystems.motor.MotorInputsAutoLogged;
import com.team236.lib.subsystems.motor.MotorSubsystemWithCanCoder;
import com.team236.lib.subsystems.motor.MotorSubsystemWithCanCoderConfig;

/**
 * The {@code ArmPivot} controls the pivoting mechanism of the robot's arm. It manages the up and
 * down positions of the arm using a motor with an integrated CANcoder for precise position control
 * and feedback.
 */
public class ArmPivotSubsystem
        extends MotorSubsystemWithCanCoder<
                MotorInputsAutoLogged, MotorIO, CanCoderInputsAutoLogged, CanCoderIO> {
    private RobotState mRobotState;
    private double mIsFullyDeployed;
    private double mIsFullyRetracted;

    public ArmPivotSubsystem(
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

        this.setCurrentPosition(Constants.ArmConstants.kArmPivotStowPositionUnits);
        this.positionSetpointUnits = Constants.ArmConstants.kArmPivotStowPositionUnits;
        this.mRobotState = state;
    }

    public void setTeleopDefaultCommand() {
        this.getDefaultCommand().cancel();
        this.setDefaultCommand(
                motionMagicSetpointCommand(this::getPositionSetpointUnits)
                        .withName("Arm Pivot Maintain Setpoint (default)")
                        .ignoringDisable(true));
    }

    public boolean isFullyDeployed() {
        return (this.mInputs.rawRotorPosition <= Constants.ArmConstants.kArmPivotRetractRadians || MathHelpers.epsilonEquals(
                                    Constants.ArmConstants.kArmPivotRetractRadians,
                                    mInputs.unitPosition,
                                    Constants.kEpsilonMotorTolerance)
);
    }

    public boolean isFullyRetract() {
        return (this.mInputs.rawRotorPosition >= Constants.ArmConstants.kArmPivotExtendRadians || MathHelpers.epsilonEquals(
                                    Constants.ArmConstants.kArmPivotExtendRadians,
                                    mInputs.unitPosition,
                                    Constants.kEpsilonMotorTolerance));
    }

    @Override
    public void periodic() {
        super.periodic();
        mRobotState.setArmPivotAngleRadians(mInputs.unitPosition);
    }
}
