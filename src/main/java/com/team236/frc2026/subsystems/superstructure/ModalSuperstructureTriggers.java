package com.team236.frc2026.subsystems.superstructure;

import com.team236.frc2026.RobotContainer;
import com.team236.frc2026.factories.ArmFactory;

public class ModalSuperstructureTriggers {
    private final RobotContainer mRobotContainer;

    public ModalSuperstructureTriggers(RobotContainer container) {
        this.mRobotContainer = container;
    }

    @SuppressWarnings("unused")
    private void configureBindings() {
        // Later can add limits in motor config
        mRobotContainer
                .getModalControls()
                .manualMode()
                .and(mRobotContainer.getModalControls().manualArmPivotExtend())
                .and(() -> !mRobotContainer.getArmPivot().isFullyDeployed())
                .onTrue(
                        ArmFactory.manualExtend(mRobotContainer)
                                .withName("ManualArmPivotExtendCommand"));

        mRobotContainer
                .getModalControls()
                .manualMode()
                .and(mRobotContainer.getModalControls().manualArmPivotRetract())
                .and(() -> !mRobotContainer.getArmPivot().isFullyRetract())
                .onTrue(
                        ArmFactory.manualRetract(mRobotContainer)
                                .withName("ManualArmPivotRetractCommand"));
    }
}
