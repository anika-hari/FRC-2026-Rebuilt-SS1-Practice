package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.Vision;

public class DriveToFuelContinuousCommand extends PIDDriveToPoseCommand {
    private static Pose2d getClosestFuel(Vision vision) {
        Pose2d globalPose = RobotState.getGlobalPose();
        Pose2d closestFuel = globalPose;
        for (Pose3d objectPose : vision.getDetectedObjects()) {
            if (closestFuel.equals(globalPose)
                    || globalPose.minus(objectPose.toPose2d()).getTranslation().getNorm()
                            < globalPose.minus(closestFuel).getTranslation().getNorm()) {
                closestFuel = objectPose.toPose2d();
            }
        }
        if (!closestFuel.equals(globalPose)) {
            return new Pose2d(
                    closestFuel.getX(),
                    closestFuel.getY(),
                    new Rotation2d(globalPose.getX() - closestFuel.getX(), globalPose.getY() - closestFuel.getY()));
        } else {
            return globalPose;
        }
    }

    public DriveToFuelContinuousCommand(RobotContainer container) {
        super(container, () -> getClosestFuel(container.getVision()));
    }
}
