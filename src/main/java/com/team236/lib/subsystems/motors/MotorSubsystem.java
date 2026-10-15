package com.team236.lib.subsystems.motors;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team236.lib.math.MathHelpers;
import com.team236.lib.roborio.RobotTime;
import edu.wpi.first.wpilibj2.command.*;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code MotorSubsystem} serves as a generic base class for all motor-based subsystems. It
 * encapsulates hardware interaction, telemetry logging, and provides standard command factories.
 *
 * @param <T> The auto-logged inputs class for the motor.
 * @param <U> The hardware IO interface for the motor.
 */
public class MotorSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO>
        extends SubsystemBase {

    protected U io;
    protected T mInputs;
    protected double mPositionSetpointUnits = 0.0;
    protected MotorSubsystemConfig mConfig;

    public MotorSubsystem(MotorSubsystemConfig config, T inputs, U io) {
        super(config.name);
        this.mConfig = config;
        this.io = io;
        this.mInputs = inputs;

        setDefaultCommand(
                dutyCycleCommand(() -> 0.0)
                        .withName(getName() + " Default Command Neutral")
                        .ignoringDisable(true));
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();
        io.readInputs(mInputs);
        Logger.processInputs(getName(), mInputs);
        Logger.recordOutput(
                getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
        Logger.recordOutput(
                getName() + "/currentCommand",
                (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
    }

    public double getCurrentPosition() {
        return mInputs.unitPosition;
    }

    public double getCurrentVelocity() {
        return mInputs.velocityUnitsPerSecond;
    }

    public double getPositionSetpointUnits() {
        return mPositionSetpointUnits;
    }

    public void setCurrentPosition(double positionUnits) {
        io.setCurrentPosition(positionUnits);
    }

    public Command setMotionMagicConfigCommand(MotionMagicConfigs configs) {
        return new InstantCommand(() -> setMotionMagicConfig(configs));
    }

    public Command dutyCycleCommand(DoubleSupplier dutyCycle) {
        return runEnd(
                        () -> {
                            setOpenLoopDutyCycleImpl(dutyCycle.getAsDouble());
                        },
                        () -> {
                            setOpenLoopDutyCycleImpl(0.0);
                        })
                .withName(getName() + " DutyCycleControl");
    }

    public Command dutyCycleCommandNoEnd(DoubleSupplier dutyCycle) {
        return runEnd(
                        () -> {
                            setOpenLoopDutyCycleImpl(dutyCycle.getAsDouble());
                        },
                        () -> {})
                .withName(getName() + " DutyCycleControl");
    }

    public Command voltageCommand(DoubleSupplier voltage) {
        return runEnd(
                        () -> {
                            setVoltageImpl(voltage.getAsDouble());
                        },
                        () -> {
                            setVoltageImpl(0.0);
                        })
                .withName(getName() + " VoltageControl");
    }

    public Command velocitySetpointCommand(DoubleSupplier velocitySupplier) {
        return velocitySetpointCommand(velocitySupplier, 0);
    }

    public Command velocitySetpointCommand(DoubleSupplier velocitySupplier, int slot) {
        return runEnd(
                        () -> {
                            setVelocitySetpointImpl(velocitySupplier.getAsDouble(), slot);
                        },
                        () -> {})
                .withName(getName() + " VelocityControl");
    }

    public Command setCoast() {
        return startEnd(
                        () -> setNeutralModeImpl(NeutralModeValue.Coast),
                        () -> setNeutralModeImpl(NeutralModeValue.Brake))
                .withName(getName() + "CoastMode")
                .ignoringDisable(true);
    }

    public Command positionSetpointCommand(DoubleSupplier unitSupplier) {
        return runEnd(
                        () -> {
                            setPositionSetpointImpl(unitSupplier.getAsDouble());
                        },
                        () -> {})
                .withName(getName() + " positionSetpointCommand");
    }

    public Command positionSetpointUntilOnTargetCommand(
            DoubleSupplier unitSupplier, DoubleSupplier epsilon) {
        return new ParallelDeadlineGroup(
                new WaitUntilCommand(
                        () ->
                                MathHelpers.epsilonEquals(
                                        unitSupplier.getAsDouble(),
                                        mInputs.unitPosition,
                                        epsilon.getAsDouble())),
                positionSetpointCommand(unitSupplier));
    }

    public Command motionMagicSetpointCommand(DoubleSupplier unitSupplier) {
        return motionMagicSetpointCommand(unitSupplier, 0);
    }

    public Command motionMagicSetpointCommand(DoubleSupplier unitSupplier, int slot) {
        return runEnd(
                        () -> {
                            setMotionMagicSetpointImpl(unitSupplier.getAsDouble(), slot);
                        },
                        () -> {})
                .withName(getName() + " motionMagicSetpointCommand");
    }

    public Command motionMagicSetpointCommand(
            DoubleSupplier unitSupplier, Supplier<MotionMagicConfigs> configSupplier) {
        return motionMagicSetpointCommand(unitSupplier, configSupplier, 0);
    }

    public Command motionMagicSetpointCommand(
            DoubleSupplier unitSupplier, Supplier<MotionMagicConfigs> configSupplier, int slot) {
        return runEnd(
                        () -> {
                            setMotionMagicSetpointImpl(
                                    unitSupplier.getAsDouble(), configSupplier.get(), slot);
                        },
                        () -> {})
                .withName(getName() + " dynamicMotionMagicSetpointCommand");
    }

    public Command motionMagicSetpointCommand(
            DoubleSupplier unitSupplier,
            Supplier<MotionMagicConfigs> configSupplier,
            Supplier<Double> feedforwards,
            int slot) {
        return runEnd(
                        () -> {
                            setMotionMagicSetpointImpl(
                                    unitSupplier.getAsDouble(),
                                    configSupplier.get(),
                                    feedforwards.get(),
                                    slot);
                        },
                        () -> {})
                .withName(getName() + " dynamicMotionMagicSetpointCommand");
    }

    public Command motionMagicSetpointCommandBlocking(DoubleSupplier setpoint, double tolerance) {
        return motionMagicSetpointCommandBlocking(setpoint, tolerance, 0);
    }

    public Command motionMagicSetpointCommandBlocking(
            DoubleSupplier setpoint, double tolerance, int slot) {
        return motionMagicSetpointCommand(() -> setpoint.getAsDouble(), slot)
                .until(
                        () ->
                                MathHelpers.epsilonEquals(
                                        getCurrentPosition(), setpoint.getAsDouble(), tolerance));
    }

    public Command motionMagicSetpointCommandBlocking(
            DoubleSupplier setpoint,
            Supplier<MotionMagicConfigs> configSupplier,
            double tolerance) {
        return motionMagicSetpointCommandBlocking(setpoint, configSupplier, tolerance, 0);
    }

    public Command motionMagicSetpointCommandBlocking(
            DoubleSupplier setpoint,
            Supplier<MotionMagicConfigs> configSupplier,
            double tolerance,
            int slot) {
        return motionMagicSetpointCommand(setpoint, configSupplier, slot)
                .until(
                        () ->
                                MathHelpers.epsilonEquals(
                                        getCurrentPosition(), setpoint.getAsDouble(), tolerance));
    }

    public Command motionMagicSetpointCommandBlocking(
            DoubleSupplier setpoint,
            Supplier<MotionMagicConfigs> configSupplier,
            Supplier<Double> feedforwards,
            double tolerance,
            int slot) {
        return motionMagicSetpointCommand(setpoint, configSupplier, feedforwards, slot)
                .until(
                        () ->
                                MathHelpers.epsilonEquals(
                                        getCurrentPosition(), setpoint.getAsDouble(), tolerance));
    }

    public Command setTorqueCurrentFOC(DoubleSupplier current) {
        return runEnd(
                        () -> {
                            setTorqueCurrentFOCImpl(current.getAsDouble());
                        },
                        () -> {})
                .withName(getName() + " torqueCurrentFOCCommand");
    }

    protected void setMotionMagicConfig(MotionMagicConfigs config) {
        io.setMotionMagicConfig(config);
    }

    protected void setOpenLoopDutyCycleImpl(double dutyCycle) {
        Logger.recordOutput(getName() + "/API/setOpenLoopDutyCycle/dutyCycle", dutyCycle);
        io.setOpenLoopDutyCycle(dutyCycle);
    }

    protected void setVoltageImpl(double voltage) {
        Logger.recordOutput(getName() + "/API/setVoltageImpl/voltage", voltage);
        io.setVoltageOutput(voltage);
    }

    protected void setPositionSetpointImpl(double units) {
        mPositionSetpointUnits = units;
        Logger.recordOutput(getName() + "/API/setPositionSetpointImp/Units", units);
        io.setPositionSetpoint(units);
    }

    protected void setNeutralModeImpl(NeutralModeValue mode) {
        Logger.recordOutput(getName() + "/API/setNeutralModeImpl/Mode", mode);
        io.setNeutralMode(mode);
    }

    protected void setMotionMagicSetpointImpl(double units, int slot) {
        mPositionSetpointUnits = units;
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImp/Units", units);
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImp/Slot", slot);
        io.setMotionMagicSetpoint(units, slot);
    }

    protected void setMotionMagicSetpointImpl(double units, MotionMagicConfigs config, int slot) {
        mPositionSetpointUnits = units;
        double velocity = config.MotionMagicCruiseVelocity;
        double acceleration = config.MotionMagicAcceleration;
        double jerk = config.MotionMagicJerk;

        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Units", units);
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Velocity", velocity);
        Logger.recordOutput(
                getName() + "/API/setMotionMagicSetpointImpDynamic/Accel", acceleration);
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Jerk", jerk);
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Slot", slot);
        io.setMotionMagicSetpoint(units, velocity, acceleration, jerk, slot);
    }

    protected void setMotionMagicSetpointImpl(
            double units, MotionMagicConfigs config, double feedfowards, int slot) {
        mPositionSetpointUnits = units;
        double velocity = config.MotionMagicCruiseVelocity;
        double acceleration = config.MotionMagicAcceleration;
        double jerk = config.MotionMagicJerk;

        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Units", units);
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Velocity", velocity);
        Logger.recordOutput(
                getName() + "/API/setMotionMagicSetpointImpDynamic/Accel", acceleration);
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Jerk", jerk);
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Slot", slot);
        Logger.recordOutput(
                getName() + "/API/setMotionMagicSetpointImpDynamic/Feedforwards", feedfowards);
        io.setMotionMagicSetpoint(units, velocity, acceleration, jerk, slot, feedfowards);
    }

    protected void setVelocitySetpointImpl(double unitsPerSecond, int slot) {
        Logger.recordOutput(getName() + "/API/setVelocitySetpointImpl/UnitsPerS", unitsPerSecond);
        io.setVelocitySetpoint(unitsPerSecond, slot);
    }

    protected void setTorqueCurrentFOCImpl(double current) {
        Logger.recordOutput(getName() + "/API/setTorqueCurrentFoC/Current", current);
        io.setTorqueCurrentFOC(current);
    }

    protected void setCurrentPositionAsZero() {
        io.setCurrentPositionAsZero();
    }

    protected Command withoutLimitsTemporarily() {
        var prev =
                new Object() {
                    boolean fwd = false;
                    boolean rev = false;
                };
        return Commands.startEnd(
                () -> {
                    prev.fwd = mConfig.talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable;
                    prev.rev = mConfig.talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable;
                    io.setEnableSoftLimits(false, false);
                },
                () -> {
                    io.setEnableSoftLimits(prev.fwd, prev.rev);
                });
    }
}
