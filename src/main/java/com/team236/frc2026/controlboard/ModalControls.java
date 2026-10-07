package com.team236.frc2026.controlboard;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code ModalControls} manages the robot's active control mode (e.g., INTAKE vs. SCORING) 
 * and handles the execution of mode-specific state change callbacks and bindings.
 */
public class ModalControls {
    private static Optional<ModalControls> mInstance = Optional.empty();

    public enum Mode {
        INTAKE,
        SCORING
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
                .getToggleMode()
                .onTrue(
                        Commands.runOnce(
                                        () -> {
                                            Mode newMode =
                                                    (mCurrentMode == Mode.INTAKE)
                                                            ? Mode.SCORING
                                                            : Mode.INTAKE;
                                            maybeTriggerStateChangeConsumer(newMode);
                                            setMode(newMode);
                                        })
                                .ignoringDisable(true));
    }

    public Trigger resetGyro() {
        return ControlBoard.getInstance().getResetGyro();
    }

    public Trigger intakeMode() {
        return new Trigger(() -> this.mCurrentMode == Mode.INTAKE);
    }

    public Trigger scoringMode() {
        return new Trigger(() -> this.mCurrentMode == Mode.SCORING);
    }

    private void maybeTriggerStateChangeConsumer(Mode newMode) {
        if (this.mCurrentMode != newMode && this.mStateChangeConsumer != null) {
            this.mStateChangeConsumer.accept(newMode);
        }
    }

    private Trigger modeSpecific(Trigger trigger, Mode mode) {
        return trigger.and(new Trigger(() -> this.mCurrentMode == mode));
    }
}