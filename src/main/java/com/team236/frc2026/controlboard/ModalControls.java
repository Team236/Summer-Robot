package com.team236.frc2026.controlboard;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code ModalControls} manages the robot's active control mode (e.g., INTAKE vs. SCORING) and
 * handles the execution of mode-specific state change callbacks and bindings.
 */
public class ModalControls {
    private static Optional<ModalControls> mInstance = Optional.empty();

    public enum Mode {
        INTAKE,
        SCORING,
        MANUAL
    }

    private Mode mCurrentMode = Mode.INTAKE;
    private Consumer<Mode> mStateChangeConsumer;

    private ModalControls() {
        Logger.recordOutput("Controls/CurrentMode", mCurrentMode.toString());
    }

    public static ModalControls getInstance() {
        if (mInstance.isEmpty()) {
            mInstance = Optional.of(new ModalControls());
        }
        return mInstance.get();
    }

    public Mode getMode() {
        return mCurrentMode;
    }

    public void setMode(Mode mode) {
        this.mCurrentMode = mode;
        Logger.recordOutput("Controls/CurrentMode", mode.toString());
    }

    public void setStateChangeConsumer(Consumer<Mode> consumer) {
        this.mStateChangeConsumer = consumer;
    }

    public void forceSetMode(Mode mode) {
        maybeTriggerStateChangeConsumer(mode);
        setMode(mode);
    }

    public void configureBindings() {
        ControlBoard.getInstance()
                .getIntakeMode()
                .onTrue(
                        Commands.runOnce(
                                        () -> {
                                            maybeTriggerStateChangeConsumer(Mode.INTAKE);
                                            setMode(Mode.INTAKE);
                                        })
                                .ignoringDisable(true));
        ControlBoard.getInstance()
                .getScoringMode()
                .onTrue(
                        Commands.runOnce(
                                        () -> {
                                            maybeTriggerStateChangeConsumer(Mode.SCORING);
                                            setMode(Mode.SCORING);
                                        })
                                .ignoringDisable(true));
        ControlBoard.getInstance()
                .getIntakeMode()
                .onTrue(
                        Commands.runOnce(
                                        () -> {
                                            maybeTriggerStateChangeConsumer(Mode.MANUAL);
                                            setMode(Mode.MANUAL);
                                        })
                                .ignoringDisable(true));
    }

    public Trigger intakeMode() {
        return new Trigger(() -> this.mCurrentMode == Mode.INTAKE);
    }

    public Trigger scoringMode() {
        return new Trigger(() -> this.mCurrentMode == Mode.SCORING);
    }

    public Trigger manualMode() {
        return new Trigger(() -> this.mCurrentMode == Mode.MANUAL);
    }

    private void maybeTriggerStateChangeConsumer(Mode newMode) {
        if (this.mCurrentMode != newMode && this.mStateChangeConsumer != null) {
            this.mStateChangeConsumer.accept(newMode);
        }
    }

    private Trigger modeSpecific(Trigger trigger, Mode mode) {
        return trigger.and(new Trigger(() -> this.mCurrentMode == mode));
    }


    // Controls //
    
    public Trigger resetGyro() {
        return ControlBoard.getInstance().getResetGyro();
    }

    // Intake Mode //
    public Trigger manualArmPivotExtend() {
        return modeSpecific(ControlBoard.getInstance().getManualArmPivotExtend(), Mode.MANUAL);
    }

    public Trigger manualArmPivotRetract() {
        return modeSpecific(ControlBoard.getInstance().getManualArmPivotRetract(), Mode.MANUAL);
    }
}
