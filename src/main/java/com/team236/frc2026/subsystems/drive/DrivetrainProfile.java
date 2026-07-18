package com.team236.frc2026.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

/**
 * The {@code DrivetrainProfile} contains the TunerX constants for the specific robot we are
 * operating. This allows us to store many TunerX files and still have one access point.
 */
public class DrivetrainProfile {
    private final SwerveDrivetrainConstants mSwerveDriveConstants;
    private final SwerveModuleConstants<?, ?, ?>[] mModuleConstants;

    public DrivetrainProfile(
            SwerveDrivetrainConstants swerveDriveConstants,
            SwerveModuleConstants<?, ?, ?>... moduleConstants) {
        this.mSwerveDriveConstants = swerveDriveConstants;
        this.mModuleConstants = moduleConstants;
    }

    public SwerveDrivetrainConstants getDrivetrainConstants() {
        return mSwerveDriveConstants;
    }

    public SwerveModuleConstants<?, ?, ?>[] getModuleConstants() {
        return mModuleConstants;
    }
}