package com.team236.frc2026.controlboard;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class ModalControls {
    private static Optional<ModalControls> instance = Optional.empty();

    public enum Mode {
        INTAKE,
        SCORING
    }

    private Mode currentMode = Mode.INTAKE;
    private Consumer<Mode> stateChangeConsumer;

    public static ModalControls getInstance() {
        if (instance.isEmpty()) {
            instance = Optional.of(new ModalControls());
        }
        return instance.get();
    }

    public Mode getMode() {
        return currentMode;
    }

    public void setMode(Mode mode) {
        this.currentMode = mode;
        Logger.recordOutput("/Controls/CurrentMode", mode.toString());
    }

    public void setStateChangeConsumer(Consumer<Mode> consumer) {
        this.stateChangeConsumer = consumer;
    }

    private void maybeTriggerStateChangeConsumer(Mode newMode) {
        if (this.currentMode != newMode && this.stateChangeConsumer != null) {
            this.stateChangeConsumer.accept(newMode);
        }
    }

    private Trigger modeSpecific(Trigger trigger, Mode mode) {
        return trigger.and(new Trigger(() -> this.currentMode == mode));
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
                                                    (currentMode == Mode.INTAKE)
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
        return new Trigger(() -> this.currentMode == Mode.INTAKE);
    }

    public Trigger scoringMode() {
        return new Trigger(() -> this.currentMode == Mode.SCORING);
    }
}
