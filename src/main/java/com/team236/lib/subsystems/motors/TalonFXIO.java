package com.team236.lib.subsystems.motors;

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
import com.team236.lib.util.CanStatusLogger;
import com.team236.lib.util.CTREUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXIO implements MotorIO {
    protected final TalonFX talon;
    protected final MotorSubsystemConfig config;

    protected final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);
    private final VelocityVoltage velocityVoltageControl = new VelocityVoltage(0.0);
    private final VoltageOut voltageControl = new VoltageOut(0.0);
    private final PositionVoltage positionVoltageControl = new PositionVoltage(0.0);
    private final MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0.0);
    private final DynamicMotionMagicVoltage dynamicMotionMagicVoltage =
            new DynamicMotionMagicVoltage(0.0, 0.0, 0.0);
    private final Follower followerControl = new Follower(0, MotorAlignmentValue.Aligned);
    private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentStatorSignal;
    private final StatusSignal<Current> currentSupplySignal;
    private final StatusSignal<Angle> rawRotorPositionSignal;

    private final BaseStatusSignal[] signals;

    public TalonFXIO(MotorSubsystemConfig config) {
        this.config = config;
        talon = new TalonFX(config.talonCANId.getDeviceId(), config.talonCANId.getDeviceNetwork());

        if (Robot.isSimulation()) {
            this.config.talonConfig.CurrentLimits = new CurrentLimitsConfigs();
            this.config.talonConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs();
            this.config.talonConfig.OpenLoopRamps = new OpenLoopRampsConfigs();
        }

        CTREUtil.applyConfiguration(talon, this.config.talonConfig);

        positionSignal = talon.getPosition();
        velocitySignal = talon.getVelocity();
        voltageSignal = talon.getMotorVoltage();
        currentStatorSignal = talon.getStatorCurrent();
        currentSupplySignal = talon.getSupplyCurrent();
        rawRotorPositionSignal = talon.getRotorPosition();

        signals =
                new BaseStatusSignal[] {
                    positionSignal, velocitySignal, voltageSignal,
                    currentStatorSignal, currentSupplySignal, rawRotorPositionSignal
                };
        CanStatusLogger.getInstance().registerTalonFX(config.name, talon, config.talonCANId);

        CTREUtil.tryUntilOK(
                () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals),
                talon.getDeviceID());
        CTREUtil.tryUntilOK(() -> talon.optimizeBusUtilization(), talon.getDeviceID());
    }

    private double rotorToUnits(double rotor) {
        return rotor * config.unitToRotorRatio;
    }

    private double clampPosition(double units) {
        return unitsToRotor(
                MathUtil.clamp(units, config.kMinPositionUnits, config.kMaxPositionUnits));
    }

    public double unitsToRotor(double units) {
        return units / config.unitToRotorRatio;
    }

    @Override
    public void readInputs(MotorInputs inputs) {
        BaseStatusSignal.refreshAll(signals);

        inputs.unitPosition = rotorToUnits(positionSignal.getValueAsDouble());
        inputs.velocityUnitsPerSecond = rotorToUnits(velocitySignal.getValueAsDouble());
        inputs.appliedVolts = voltageSignal.getValueAsDouble();
        inputs.currentStatorAmps = currentStatorSignal.getValueAsDouble();
        inputs.currentSupplyAmps = currentSupplySignal.getValueAsDouble();
        inputs.rawRotorPosition = rawRotorPositionSignal.getValueAsDouble();
    }

    @Override
    public void setOpenLoopDutyCycle(double dutyCycle) {
        talon.setControl(dutyCycleControl.withOutput(dutyCycle));
    }

    @Override
    public void setPositionSetpoint(double units) {
        talon.setControl(positionVoltageControl.withPosition(clampPosition(units)));
    }

    @Override
    public void setMotionMagicConfig(MotionMagicConfigs config) {
        this.config.talonConfig.MotionMagic = config;
        CTREUtil.applyConfiguration(talon, this.config.talonConfig.MotionMagic);
    }

    @Override
    public void setMotionMagicSetpoint(double units, int slot) {
        talon.setControl(
                motionMagicPositionControl.withPosition(clampPosition(units)).withSlot(slot));
    }

    @Override
    public void setMotionMagicSetpoint(
            double units,
            double velocity,
            double acceleration,
            double jerk,
            int slot,
            double feedfowards) {
        talon.setControl(
                dynamicMotionMagicVoltage
                        .withPosition(clampPosition(units))
                        .withAcceleration(acceleration)
                        .withJerk(jerk)
                        .withVelocity(velocity)
                        .withSlot(slot)
                        .withFeedForward(feedfowards));
    }

    @Override
    public void setNeutralMode(NeutralModeValue mode) {
        config.talonConfig.MotorOutput.NeutralMode = mode;
        CTREUtil.applyConfiguration(talon, config.talonConfig);
    }

    @Override
    public void setEnableSoftLimits(boolean fwd, boolean rev) {
        config.talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = fwd;
        config.talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = rev;
        CTREUtil.applyConfiguration(talon, config.talonConfig);
    }

    @Override
    public void setEnableHardLimits(boolean fwd, boolean rev) {
        config.talonConfig.HardwareLimitSwitch.ForwardLimitEnable = fwd;
        config.talonConfig.HardwareLimitSwitch.ReverseLimitEnable = rev;
        CTREUtil.applyConfiguration(talon, config.talonConfig.HardwareLimitSwitch);
    }

    @Override
    public void setEnableAutosetPositionValue(boolean fwd, boolean rev) {
        config.talonConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = fwd;
        config.talonConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = rev;
        config.talonConfig.HardwareLimitSwitch.ForwardLimitEnable = fwd;
        config.talonConfig.HardwareLimitSwitch.ReverseLimitEnable = rev;
        CTREUtil.applyConfiguration(talon, config.talonConfig.HardwareLimitSwitch);
    }

    @Override
    public void setVelocitySetpoint(double unitsPerSecond, int slot) {
        talon.setControl(
                velocityVoltageControl.withVelocity(unitsToRotor(unitsPerSecond)).withSlot(slot));
    }

    @Override
    public void setCurrentPositionAsZero() {
        setCurrentPosition(0.0);
    }

    @Override
    public void setCurrentPosition(double positionUnits) {
        talon.setPosition(unitsToRotor(positionUnits));
    }

    @Override
    public void setVoltageOutput(double voltage) {
        talon.setControl(voltageControl.withOutput(voltage));
    }

    @Override
    public void follow(CanDeviceId leaderId, MotorAlignmentValue direction) {
        CTREUtil.tryUntilOK(
                () ->
                        talon.setControl(
                                followerControl
                                        .withLeaderID(leaderId.getDeviceId())
                                        .withMotorAlignment(direction)),
                this.config.talonCANId.getDeviceId());
    }

    @Override
    public void setTorqueCurrentFOC(double current) {
        talon.setControl(torqueCurrentFOC.withOutput(current));
    }

    @Override
    public void setVoltageConfig(VoltageConfigs inputConfig) {
        CTREUtil.applyConfigurationNonBlocking(talon, inputConfig);
    }
}
