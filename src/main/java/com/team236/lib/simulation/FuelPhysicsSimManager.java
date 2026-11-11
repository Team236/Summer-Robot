package com.team236.lib.simulation;

import com.team236.frc2026.Constants;
import com.team236.frc2026.RobotContainer;
import edu.wpi.first.math.util.Units;

public class FuelPhysicsSimManager {
    private static FuelPhysicsSim mFuelSim;

    public static void configureFuelSim(RobotContainer container) {
        mFuelSim =
                FuelPhysicsSim.configureFuelSim(
                        Constants.TestbedConstants.kBumperWidthInches,
                        Constants.TestbedConstants.kBumperLengthInches,
                        Constants.TestbedConstants.kBumperHeightInches,
                        () -> container.getSimulatedRobotState().getLatestFieldToRobot(),
                        () -> container.getSimulatedRobotState().getSimChassisSpeeds());

        mFuelSim.addIntakeZone(
                () -> container.getRobotState().getArmPose(),
                Units.inchesToMeters(7.0),
                Units.inchesToMeters(7.0),
                () -> {
                    if (container.getRobotState().getIntakePossession()
                            || !container.getArmIntake().isIntaking()) {
                        return false;
                    }
                    return true;
                },
                () -> container.getRobotState().setIntakePossession(true));
    }

    public static void tick() {
        if (mFuelSim != null) {
            mFuelSim.tick();
        }
    }

    public static void launchFuel() {
        mFuelSim.launchBall(null, null, null);
    }
}
