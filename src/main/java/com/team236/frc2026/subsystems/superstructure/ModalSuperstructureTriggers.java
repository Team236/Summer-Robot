package com.team236.frc2026.subsystems.superstructure;

import com.team236.frc2026.RobotContainer;
import com.team236.frc2026.factories.ArmIntakeFactory;
import com.team236.frc2026.factories.ArmPivotFactory;

public class ModalSuperstructureTriggers {
    private final RobotContainer mRobotContainer;

    public ModalSuperstructureTriggers(RobotContainer container) {
        this.mRobotContainer = container;
        configureTriggers();
    }

    @SuppressWarnings("unused")
    private void configureTriggers() {
        // Later can add limits in motor config
        mRobotContainer
                .getModalControls()
                .manualMode()
                .and(mRobotContainer.getModalControls().manualArmPivotExtend())
                .and(() -> !mRobotContainer.getArmPivot().isFullyDeployed())
                .whileTrue(
                        ArmPivotFactory.manualExtend(mRobotContainer)
                                .withName("ManualArmPivotExtendCommand"));

        mRobotContainer
                .getModalControls()
                .manualMode()
                .and(mRobotContainer.getModalControls().manualArmPivotRetract())
                .and(() -> !mRobotContainer.getArmPivot().isFullyRetract())
                .whileTrue(
                        ArmPivotFactory.manualRetract(mRobotContainer)
                                .withName("ManualArmPivotRetractCommand"));

        mRobotContainer
                .getModalControls()
                .manualArmIntake()
                .whileTrue(
                        ArmIntakeFactory.manualIntake((mRobotContainer))
                                .withName("ManualArmIntakeCommand"));
        mRobotContainer
                .getModalControls()
                .manualArmOuttake()
                .whileTrue(
                        ArmIntakeFactory.manualOuttake((mRobotContainer))
                                .withName("ManualArmOuttakeCommand"));
    }
}
