package com.team236.lib.simulation;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

/**
 * The {@code MapleSimSwerveDrivetrain} retrieves simulation data from Maple-Sim and injects it into
 * the CTRE SwerveDrivetrain instance, replacing the default SimSwerveDrivetrain class.
 */
public class MapleSimSwerveDrivetrain {
    private final Pigeon2SimState mPigeonSim;
    private final SimSwerveModule[] mSimModules;
    public final SwerveDriveSimulation mapleSimDrive;

    /**
     * Constructs a drivetrain simulation using the specified parameters.
     *
     * @param simPeriod the time period of the simulation
     * @param robotMassWithBumpers the total mass of the robot, including bumpers
     * @param bumperLengthX the length of the bumper along the X-axis (influences the collision
     *     space of the robot)
     * @param bumperWidthY the width of the bumper along the Y-axis (influences the collision space
     *     of the robot)
     * @param driveMotorModel the {@link DCMotor} model for the drive motor, typically <code>
     *        DCMotor.getKrakenX60Foc()
     *        </code>
     * @param steerMotorModel the {@link DCMotor} model for the steer motor, typically <code>
     *        DCMotor.getKrakenX60Foc()
     *        </code>
     * @param wheelCOF the coefficient of friction of the drive wheels
     * @param moduleLocations the locations of the swerve modules on the robot, in the order <code>
     *        FL, FR, BL, BR</code>
     * @param pigeon the {@link Pigeon2} IMU used in the drivetrain
     * @param modules the {@link SwerveModule}s, typically obtained via {@link
     *     SwerveDrivetrain#getModules()}
     * @param moduleConstants the constants for the swerve modules
     */
    public MapleSimSwerveDrivetrain(
            Time simPeriod,
            Mass robotMassWithBumpers,
            Distance bumperLengthX,
            Distance bumperWidthY,
            DCMotor driveMotorModel,
            DCMotor steerMotorModel,
            double wheelCOF,
            Translation2d[] moduleLocations,
            Pigeon2 pigeon,
            SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
            @SuppressWarnings("unchecked")
                    SwerveModuleConstants<
                                    TalonFXConfiguration,
                                    TalonFXConfiguration,
                                    CANcoderConfiguration>...
                            moduleConstants) {
        this.mPigeonSim = pigeon.getSimState();
        mSimModules = new SimSwerveModule[moduleConstants.length];
        DriveTrainSimulationConfig simulationConfig =
                DriveTrainSimulationConfig.Default()
                        .withRobotMass(robotMassWithBumpers)
                        .withBumperSize(bumperLengthX, bumperWidthY)
                        .withGyro(COTS.ofPigeon2())
                        .withCustomModuleTranslations(moduleLocations)
                        .withSwerveModule(
                                new SwerveModuleSimulationConfig(
                                        driveMotorModel,
                                        steerMotorModel,
                                        moduleConstants[0].DriveMotorGearRatio,
                                        moduleConstants[0].SteerMotorGearRatio,
                                        Volts.of(moduleConstants[0].DriveFrictionVoltage),
                                        Volts.of(moduleConstants[0].SteerFrictionVoltage),
                                        Meters.of(moduleConstants[0].WheelRadius),
                                        KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
                                        wheelCOF));
        mapleSimDrive = new SwerveDriveSimulation(simulationConfig, new Pose2d());

        SwerveModuleSimulation[] moduleSimulations = mapleSimDrive.getModules();
        for (int i = 0; i < mSimModules.length; i++) {
            mSimModules[i] =
                    new SimSwerveModule(moduleConstants[0], moduleSimulations[i], modules[i]);
        }

        SimulatedArena.overrideSimulationTimings(simPeriod, 1);

        // This turns a barrier wall on or off for the ramp area, also eff mode is if 400+ balls
        // spawn instead of 100
        Arena2026Rebuilt arena = new Arena2026Rebuilt(false);
        arena.setEfficiencyMode(false);
        SimulatedArena.overrideInstance(arena);
        SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
    }

    public static class TalonFXMotorControllerWithRemoteCanCoderSim
            extends TalonFXMotorControllerSim {
        private final CANcoderSimState mRemoteCancoderSimState;

        public TalonFXMotorControllerWithRemoteCanCoderSim(TalonFX talonFX, CANcoder cancoder) {
            super(talonFX);
            this.mRemoteCancoderSimState = cancoder.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            mRemoteCancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            mRemoteCancoderSimState.setRawPosition(mechanismAngle);
            mRemoteCancoderSimState.setVelocity(mechanismVelocity);

            return super.updateControlSignal(
                    mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
        }
    }

    // Static utils classes
    public static class TalonFXMotorControllerSim implements SimulatedMotorController {
        public final int id;

        private final TalonFXSimState mTalonFXSimState;

        public TalonFXMotorControllerSim(TalonFX talonFX) {
            this.id = talonFX.getDeviceID();
            this.mTalonFXSimState = talonFX.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            mTalonFXSimState.setRawRotorPosition(encoderAngle);
            mTalonFXSimState.setRotorVelocity(encoderVelocity);
            mTalonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

            return mTalonFXSimState.getMotorVoltageMeasure();
        }
    }

    /**
     * Updates the Maple-Sim simulation and injects the results into the simulated CTRE devices,
     * including motors and the IMU.
     */
    public void update() {
        SimulatedArena.getInstance().simulationPeriodic();
        mPigeonSim.setRawYaw(mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
        mPigeonSim.setAngularVelocityZ(
                RadiansPerSecond.of(
                        mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative()
                                .omegaRadiansPerSecond));
    }

    /**
     * Regulates all {@link SwerveModuleConstants} for a drivetrain simulation.
     *
     * <p>This method processes an array of {@link SwerveModuleConstants} to apply necessary
     * adjustments for simulation purposes, ensuring compatibility and avoiding known bugs.
     *
     * @see #regulateModuleConstantForSimulation(SwerveModuleConstants)
     */
    public static SwerveModuleConstants<?, ?, ?>[] regulateModuleConstantsForSimulation(
            SwerveModuleConstants<?, ?, ?>[] moduleConstants) {
        for (SwerveModuleConstants<?, ?, ?> moduleConstant : moduleConstants) {
            regulateModuleConstantForSimulation(moduleConstant);
        }
        return moduleConstants;
    }

    /** The {@code SimSwerveModule} represents the simulation of a single SwerveModule. */
    protected static class SimSwerveModule {
        public final SwerveModuleConstants<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                moduleConstant;
        public final SwerveModuleSimulation moduleSimulation;

        public SimSwerveModule(
                SwerveModuleConstants<
                                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                        moduleConstant,
                SwerveModuleSimulation moduleSimulation,
                SwerveModule<TalonFX, TalonFX, CANcoder> module) {
            this.moduleConstant = moduleConstant;
            this.moduleSimulation = moduleSimulation;
            moduleSimulation.useDriveMotorController(
                    new TalonFXMotorControllerSim(module.getDriveMotor()));
            moduleSimulation.useSteerMotorController(
                    new TalonFXMotorControllerWithRemoteCanCoderSim(
                            module.getSteerMotor(), module.getEncoder()));
        }
    }

    /**
     * Regulates the {@link SwerveModuleConstants} for a single module.
     *
     * <p>This method applies specific adjustments to the {@link SwerveModuleConstants} for
     * simulation purposes. These changes have no effect on real robot operations and address known
     * simulation bugs.
     */
    private static void regulateModuleConstantForSimulation(
            SwerveModuleConstants<?, ?, ?> moduleConstants) {
        // Skip regulation if running on a real robot
        if (RobotBase.isReal()) return;

        // Apply simulation-specific adjustments to module constants
        moduleConstants
                // Disable encoder offsets
                .withEncoderOffset(0)
                // Disable motor inversions for drive and steer motors
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(false)
                // Disable CanCoder inversion
                .withEncoderInverted(false)
                // Adjust steer motor PID gains for simulation
                .withSteerMotorGains(
                        moduleConstants
                                .SteerMotorGains
                                .withKP(70) // Proportional gain
                                .withKD(4.5)) // Derivative gain
                // Adjust friction voltages
                .withDriveFrictionVoltage(Volts.of(0.1))
                .withSteerFrictionVoltage(Volts.of(0.15))
                // Adjust steer inertia
                .withSteerInertia(KilogramSquareMeters.of(0.05));
    }
}
