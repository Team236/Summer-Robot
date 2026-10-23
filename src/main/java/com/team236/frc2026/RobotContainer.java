package com.team236.frc2026;

import com.team236.frc2026.commands.TeleopSwerveDrive;
import com.team236.frc2026.controlboard.ControlBoard;
import com.team236.frc2026.controlboard.ModalControls;
import com.team236.frc2026.simulation.SimulatedRobotState;
import com.team236.frc2026.subsystems.arm.ArmPivotSubsystem;
import com.team236.frc2026.subsystems.drive.DriveHardware;
import com.team236.frc2026.subsystems.drive.DriveSim;
import com.team236.frc2026.subsystems.drive.DriveSubsystem;
import com.team236.frc2026.subsystems.superstructure.ModalSuperstructureTriggers;
import com.team236.frc2026.subsystems.vision.VisionFieldPoseEstimate;
import com.team236.frc2026.subsystems.vision.VisionHardwareLimelight;
import com.team236.frc2026.subsystems.vision.VisionSimPhoton;
import com.team236.frc2026.subsystems.vision.VisionSubsystem;
import com.team236.lib.math.MathHelpers;
import com.team236.lib.subsystems.cancoder.CanCoderHardware;
import com.team236.lib.subsystems.cancoder.SimCanCoderIO;
import com.team236.lib.subsystems.motor.SimTalonFXWithCancoder;
import com.team236.lib.subsystems.motor.TalonFXIO;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Consumer;

/**
 * The {@code RobotContainer} class holds robot subsystems, commands, and operator interface
 * bindings. It acts as the primary structure for dependency injection and button mapping.
 */
public class RobotContainer {

    private final Consumer<VisionFieldPoseEstimate> mVisionEstimateConsumer =
            new Consumer<VisionFieldPoseEstimate>() {
                @Override
                public void accept(VisionFieldPoseEstimate estimate) {
                    mDriveSubsystem.addVisionMeasurement(estimate);
                }
            };

    private final RobotState mRobotState = new RobotState(mVisionEstimateConsumer);
    private final SimulatedRobotState mSimulatedRobotState = new SimulatedRobotState(this);
    private final DriveSubsystem mDriveSubsystem = buildDriveSubsystem();
    private final VisionSubsystem mVisionSubsystem = buildVisionSubsystem();
    private final ArmPivotSubsystem mArmPivotSubsystem = buildArmPivotSubsystem();

    private final ModalControls mModalControls = ModalControls.getInstance();
    private ModalSuperstructureTriggers mModalSuperstructureTriggers;

    private final TeleopSwerveDrive mDriveCommand =
            new TeleopSwerveDrive(
                    mDriveSubsystem,
                    mRobotState,
                    this,
                    () -> ControlBoard.getInstance().getThrottle(),
                    () -> ControlBoard.getInstance().getStrafe(),
                    () -> ControlBoard.getInstance().getRotation());

    public RobotContainer() {
        configureBindings();

        if (Robot.isSimulation()) {
            assert this.mSimulatedRobotState != null;
            mSimulatedRobotState.init();
        }
    }

    private void configureBindings() {
        mModalControls.configureBindings();
        mModalSuperstructureTriggers = new ModalSuperstructureTriggers(this);
        mDriveSubsystem.setDefaultCommand(mDriveCommand);

        ControlBoard.getInstance()
                .getResetGyro()
                .onTrue(Commands.runOnce(() -> this.resetHeading()));
    }

    public void resetHeading() {
        mDriveSubsystem.resetOdometry(
                new Pose2d(
                        new Translation2d(
                                mRobotState.getLatestFieldToRobot().getValue().getX(),
                                mRobotState.getLatestFieldToRobot().getValue().getY()),
                        mRobotState.isRedAlliance()
                                ? MathHelpers.kRotation2dPi
                                : MathHelpers.kRotation2dZero));
    }

    public DriveSubsystem getDriveSubsystem() {
        return mDriveSubsystem;
    }

    public ArmPivotSubsystem getArmPivot() {
        return mArmPivotSubsystem;
    }

    public SimulatedRobotState getSimulatedRobotState() {
        return mSimulatedRobotState;
    }

    public ModalControls getModalControls() {
        return mModalControls;
    }

    public ModalSuperstructureTriggers getModalSuperstructureTriggers() {
        return mModalSuperstructureTriggers;
    }

    private DriveSubsystem buildDriveSubsystem() {
        if (Constants.kCurrentMode == Constants.Mode.SIM) {
            return new DriveSubsystem(
                    new DriveSim(
                            mRobotState,
                            mSimulatedRobotState,
                            Constants.DriveConstants.kDrivetrain.getDrivetrainConstants(),
                            Constants.DriveConstants.kDrivetrain.getModuleConstants()));
        } else {
            return new DriveSubsystem(
                    new DriveHardware(
                            mRobotState,
                            Constants.DriveConstants.kDrivetrain.getDrivetrainConstants(),
                            Constants.DriveConstants.kDrivetrain.getModuleConstants()));
        }
    }

    private VisionSubsystem buildVisionSubsystem() {
        if (Constants.kCurrentMode == Constants.Mode.SIM) {
            return new VisionSubsystem(
                    new VisionSimPhoton(mRobotState, mSimulatedRobotState), mRobotState);
        } else {
            return new VisionSubsystem(new VisionHardwareLimelight(mRobotState), mRobotState);
        }
    }

    private ArmPivotSubsystem buildArmPivotSubsystem() {
        if (RobotBase.isSimulation()) {
            var simTalon = new SimTalonFXWithCancoder(Constants.ArmConstants.kArmPivotConfig);
            return new ArmPivotSubsystem(
                    Constants.ArmConstants.kArmPivotConfig,
                    simTalon,
                    new SimCanCoderIO(
                            Constants.ArmConstants.kArmPivotConfig.canCoderConfig,
                            simTalon.getSupplierForCancoder(
                                    Constants.ArmConstants.kArmPivotConfig)),
                    mRobotState);
        } else {
            return new ArmPivotSubsystem(
                    Constants.ArmConstants.kArmPivotConfig,
                    new TalonFXIO(Constants.ArmConstants.kArmPivotConfig),
                    new CanCoderHardware(Constants.ArmConstants.kArmPivotConfig.canCoderConfig),
                    mRobotState);
        }
    }
}
