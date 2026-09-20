package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.BLine.FlippingUtil;
import frc.lib.BLine.Path;
import frc.lib.util.Util;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionConstants;

public class DoubleSwipeFar extends BaseAuto {

    public DoubleSwipeFar(RobotContainer container) {
        super(container);
    }

    Path path_1 = new Path("2swipe_first_scoop");
    Path path_2 = new Path("2swipe_second_lowerCenter");

    public Command getAutoCommand() {
        RobotState.isDoneShooting = false;
        return Commands.sequence(
                        new InstantCommand(() -> {
                            Pose2d startPose = path_1.getStartPose();
                            if (Util.isRedSide()) {
                                startPose = FlippingUtil.flipFieldPose(startPose);
                            }
                            drive.resetOdometry(RobotState.getVisionPose(VisionConstants.visionPoseThresholdSeconds)
                                    .orElse(startPose));
                            RobotState.isDoneShooting = false;
                        }),
                        drive.followPathWithSOTMHubRotation(path_1, 1).withName("Path 1"),
                        Commands.deadline(Commands.waitUntil(() -> RobotState.isDoneShooting), drive.autoRotateToHub()),
                        drive.followPathWithSOTMHubRotation(path_2, 1.5).withName("Path 2"),
                        drive.autoRotateToHub())

                // Commands.deadline(Commands.waitSeconds(4), getAutoRotateCommand()),
                // drive.getPathBuilder().build(path_3).withName("climb"))
                .withName("Entire sequence");
    }
}
