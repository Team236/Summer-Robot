package com.team236.lib.subsystems.motor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team236.frc2026.Robot;
import com.team236.lib.subsystems.cancoder.CanDeviceId;
import com.team236.lib.util.CTREUtil;
import com.team236.lib.util.CanStatusLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * The {@code TalonFXIO} manages the hardware interactions and telemetry logic for a TalonFX motor.
 */
public class TalonFXIO implements MotorIO {
    private final TalonFX mTalon;
    private final MotorSubsystemConfig mConfig;

    private final DutyCycleOut mDutyCycleControl = new DutyCycleOut(0.0);
    private final VelocityVoltage mVelocityVoltageControl = new VelocityVoltage(0.0);
    private final VoltageOut mVoltageControl = new VoltageOut(0.0);
    private final PositionVoltage mPositionVoltageControl = new PositionVoltage(0.0);
    private final MotionMagicVoltage mMotionMagicPositionControl = new MotionMagicVoltage(0.0);
    private final DynamicMotionMagicVoltage mDynamicMotionMagicVoltage =
            new DynamicMotionMagicVoltage(0.0, 0.0, 0.0);
    private final Follower mFollowerControl = new Follower(0, MotorAlignmentValue.Aligned);
    private final TorqueCurrentFOC mTorqueCurrentFOC = new TorqueCurrentFOC(0.0);

    private final StatusSignal<Angle> mPositionSignal;
    private final StatusSignal<AngularVelocity> mVelocitySignal;
    private final StatusSignal<Voltage> mVoltageSignal;
    private final StatusSignal<Current> mCurrentStatorSignal;
    private final StatusSignal<Current> mCurrentSupplySignal;
    private final StatusSignal<Angle> mRawRotorPositionSignal;

    private final BaseStatusSignal[] mSignals;

    public TalonFXIO(MotorSubsystemConfig config) {
        this.mConfig = config;
        mTalon =
                new TalonFX(
                        mConfig.talonCANId.getDeviceId(), mConfig.talonCANId.getDeviceNetwork());

        if (Robot.isSimulation()) {
            this.mConfig.talonConfig.CurrentLimits = new CurrentLimitsConfigs();
            this.mConfig.talonConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs();
            this.mConfig.talonConfig.OpenLoopRamps = new OpenLoopRampsConfigs();
        }

        CTREUtil.applyConfiguration(mTalon, this.mConfig.talonConfig);

        mPositionSignal = mTalon.getPosition();
        mVelocitySignal = mTalon.getVelocity();
        mVoltageSignal = mTalon.getMotorVoltage();
        mCurrentStatorSignal = mTalon.getStatorCurrent();
        mCurrentSupplySignal = mTalon.getSupplyCurrent();
        mRawRotorPositionSignal = mTalon.getRotorPosition();

        mSignals =
                new BaseStatusSignal[] {
                    mPositionSignal, mVelocitySignal, mVoltageSignal,
                    mCurrentStatorSignal, mCurrentSupplySignal, mRawRotorPositionSignal
                };
        CanStatusLogger.getInstance().registerTalonFX(mConfig.name, mTalon, mConfig.talonCANId);

        CTREUtil.tryUntilOK(
                () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, mSignals),
                mTalon.getDeviceID());
        CTREUtil.tryUntilOK(() -> mTalon.optimizeBusUtilization(), mTalon.getDeviceID());
    }

    public double unitsToRotor(double units) {
        return units / mConfig.unitToRotorRatio;
    }

    @Override
    public void readInputs(MotorInputs inputs) {
        BaseStatusSignal.refreshAll(mSignals);

        inputs.unitPosition = rotorToUnits(mPositionSignal.getValueAsDouble());
        inputs.velocityUnitsPerSecond = rotorToUnits(mVelocitySignal.getValueAsDouble());
        inputs.appliedVolts = mVoltageSignal.getValueAsDouble();
        inputs.currentStatorAmps = mCurrentStatorSignal.getValueAsDouble();
        inputs.currentSupplyAmps = mCurrentSupplySignal.getValueAsDouble();
        inputs.rawRotorPosition = mRawRotorPositionSignal.getValueAsDouble();
    }

    @Override
    public void setOpenLoopDutyCycle(double dutyCycle) {
        mTalon.setControl(mDutyCycleControl.withOutput(dutyCycle));
    }

    @Override
    public void setPositionSetpoint(double units) {
        mTalon.setControl(mPositionVoltageControl.withPosition(clampPosition(units)));
    }

    @Override
    public void setMotionMagicConfig(MotionMagicConfigs config) {
        this.mConfig.talonConfig.MotionMagic = config;
        CTREUtil.applyConfiguration(mTalon, this.mConfig.talonConfig.MotionMagic);
    }

    @Override
    public void setMotionMagicSetpoint(double units, int slot) {
        mTalon.setControl(
                mMotionMagicPositionControl.withPosition(clampPosition(units)).withSlot(slot));
    }

    @Override
    public void setMotionMagicSetpoint(
            double units,
            double velocity,
            double acceleration,
            double jerk,
            int slot,
            double feedfowards) {
        mTalon.setControl(
                mDynamicMotionMagicVoltage
                        .withPosition(clampPosition(units))
                        .withAcceleration(acceleration)
                        .withJerk(jerk)
                        .withVelocity(velocity)
                        .withSlot(slot)
                        .withFeedForward(feedfowards));
    }

    @Override
    public void setNeutralMode(NeutralModeValue mode) {
        mConfig.talonConfig.MotorOutput.NeutralMode = mode;
        CTREUtil.applyConfiguration(mTalon, mConfig.talonConfig);
    }

    @Override
    public void setEnableSoftLimits(boolean fwd, boolean rev) {
        mConfig.talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = fwd;
        mConfig.talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = rev;
        CTREUtil.applyConfiguration(mTalon, mConfig.talonConfig);
    }

    @Override
    public void setEnableHardLimits(boolean fwd, boolean rev) {
        mConfig.talonConfig.HardwareLimitSwitch.ForwardLimitEnable = fwd;
        mConfig.talonConfig.HardwareLimitSwitch.ReverseLimitEnable = rev;
        CTREUtil.applyConfiguration(mTalon, mConfig.talonConfig.HardwareLimitSwitch);
    }

    @Override
    public void setEnableAutosetPositionValue(boolean fwd, boolean rev) {
        mConfig.talonConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = fwd;
        mConfig.talonConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = rev;
        mConfig.talonConfig.HardwareLimitSwitch.ForwardLimitEnable = fwd;
        mConfig.talonConfig.HardwareLimitSwitch.ReverseLimitEnable = rev;
        CTREUtil.applyConfiguration(mTalon, mConfig.talonConfig.HardwareLimitSwitch);
    }

    @Override
    public void setVelocitySetpoint(double unitsPerSecond, int slot) {
        mTalon.setControl(
                mVelocityVoltageControl.withVelocity(unitsToRotor(unitsPerSecond)).withSlot(slot));
    }

    @Override
    public void setCurrentPositionAsZero() {
        setCurrentPosition(0.0);
    }

    @Override
    public void setCurrentPosition(double positionUnits) {
        mTalon.setPosition(unitsToRotor(positionUnits));
    }

    @Override
    public void setVoltageOutput(double voltage) {
        mTalon.setControl(mVoltageControl.withOutput(voltage));
    }

    @Override
    public void follow(CanDeviceId leaderId, MotorAlignmentValue direction) {
        CTREUtil.tryUntilOK(
                () ->
                        mTalon.setControl(
                                mFollowerControl
                                        .withLeaderID(leaderId.getDeviceId())
                                        .withMotorAlignment(direction)),
                this.mConfig.talonCANId.getDeviceId());
    }

    @Override
    public void setTorqueCurrentFOC(double current) {
        mTalon.setControl(mTorqueCurrentFOC.withOutput(current));
    }

    @Override
    public void setVoltageConfig(VoltageConfigs inputConfig) {
        CTREUtil.applyConfigurationNonBlocking(mTalon, inputConfig);
    }

    private double rotorToUnits(double rotor) {
        return rotor * mConfig.unitToRotorRatio;
    }

    private double clampPosition(double units) {
        return unitsToRotor(
                MathUtil.clamp(units, mConfig.kMinPositionUnits, mConfig.kMaxPositionUnits));
    }
}
