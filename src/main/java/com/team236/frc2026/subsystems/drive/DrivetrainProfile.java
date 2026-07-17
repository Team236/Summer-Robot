package com.team236.frc2026.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

/**
 * The {@code DrivetrainConfig} contains the TunerX constants for the specific robot we are
 * operating. This allows us to store many TunerX files and still have one access point.
 */
public class DrivetrainProfile {
    SwerveDrivetrainConstants swerveDriveConstants;
    SwerveModuleConstants<?, ?, ?>[] moduleConstants;

    DrivetrainProfile(
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
