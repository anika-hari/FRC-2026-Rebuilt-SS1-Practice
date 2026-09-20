package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class DriveToFuelCommand extends Command {

    private Pose2d getClosestFuel(Vision vision) {
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

    private PIDDriveToPoseCommand pidDriveToPoseCommand;
    private Pose2d targetPose;
    private final RobotContainer container;

    public DriveToFuelCommand(RobotContainer container) {
        this.container = container;
    }

    @Override
    public void initialize() {
        targetPose = getClosestFuel(container.getVision());
        pidDriveToPoseCommand = new PIDDriveToPoseCommand(container, () -> targetPose);
        pidDriveToPoseCommand.initialize();
    }

    @Override
    public void execute() {
        pidDriveToPoseCommand.execute();
        Logger.recordOutput("Commands/DriveToFuelCommand/closestFuel", getClosestFuel(container.getVision()));
    }

    @Override
    public boolean isFinished() {
        return pidDriveToPoseCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        pidDriveToPoseCommand.end(interrupted);
    }
}
