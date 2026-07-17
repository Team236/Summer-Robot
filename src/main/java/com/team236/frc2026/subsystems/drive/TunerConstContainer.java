package com.team236.frc2026.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

/**
 * The {@code TunerConstContainer} contains the TunerX constants for the specific robot we are
 * operating. This allows us to store many TunerX files and still have one access point.
 */
public class TunerConstContainer {
    SwerveDrivetrainConstants swerveDriveConstants;
    SwerveModuleConstants<?, ?, ?>[] moduleConstants;

    TunerConstContainer(
            SwerveDrivetrainConstants swerveDriveConstants,
            SwerveModuleConstants<?, ?, ?>... moduleConstants) {
        this.swerveDriveConstants = swerveDriveConstants;
        this.moduleConstants = moduleConstants;
    }

    public SwerveDrivetrainConstants getDrivetrainConstants() {
        return swerveDriveConstants;
    }

    public SwerveModuleConstants<?, ?, ?>[] getModuleConstants() {
        return moduleConstants;
    }
}
