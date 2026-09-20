package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.BLine.FlippingUtil;
import frc.lib.BLine.Path;
import frc.lib.BLine.Path.PathConstraints;
import frc.lib.BLine.Path.RangedConstraint;
import frc.lib.util.Util;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionConstants;

public class hpsToDepot extends BaseAuto {

    public hpsToDepot(RobotContainer container) {
        super(container);
    }

    Path bump_to_HPS = new Path("bump_to_HPS");
    Path HPS_to_Depot = new Path("HPS_to_Depot");
    PathConstraints constraints = new PathConstraints().setMaxVelocityMetersPerSec(new RangedConstraint(1.25, 4, 4));

    public Command getAutoCommand() {
        // loop.setPathConstraints(constraints);
        return Commands.sequence(
                        new InstantCommand(() -> {
                            Pose2d startPose = bump_to_HPS.getStartPose();
                            if (Util.isRedSide()) {
                                startPose = FlippingUtil.flipFieldPose(startPose);
                            }
                            drive.resetOdometry(RobotState.getVisionPose(VisionConstants.visionPoseThresholdSeconds)
                                    .orElse(startPose));
                        }),
                        drive.followPathWithHubRotation(bump_to_HPS, 0)
                                .withName("Bump to HPS")
                                .withTimeout(1.08 + 5),
                        drive.followPathWithHubRotation(HPS_to_Depot, 2)
                                .withName("HPS to Depot")
                                .withTimeout(2.74 + 4),
                        drive.getPathBuilder().build(HPS_to_Depot))
                .withName("Entire sequence");
    }
}
