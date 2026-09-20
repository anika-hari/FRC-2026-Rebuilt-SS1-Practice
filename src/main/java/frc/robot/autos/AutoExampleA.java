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

public class AutoExampleA extends BaseAuto {
    public AutoExampleA(RobotContainer container) {
        super(container);
    }

    Path path_3 = new Path("auto_1_climb");
    Path path_1 = new Path("auto_1_path_1");
    Path path_2 = new Path("auto_1_path_2");
    PathConstraints constraints = new PathConstraints().setMaxVelocityMetersPerSec(new RangedConstraint(1.25, 3, 4));

    Path path_test = new Path("overBump");

    // private Command getAutoRotateCommand() {

    //     return Commands.parallel(
    //             shooterManager.startShooting(),
    //             feederManager.startFeederShoot(),
    //             drive.autoRotateToHub(),
    //             Commands.sequence(
    //                             Commands.waitUntil(() -> (shooter.isAtSetpoint()
    //                                     && (feeder.isAtSetpoint())
    //                                     && drive.isAutoAlignAtSetpoint())),
    //                             Commands.parallel(hopperManager.runHopper(), intakeManager.pushIntake())
    //                                     .until(() -> !shooter.isAtSetpoint()
    //                                             || !feeder.isAtSetpoint()
    //                                             || !drive.isAutoAlignAtSetpoint()))
    //                     .repeatedly()
    //                     .finallyDo(() -> {
    //                         shooter.setShooting(false);
    //                         feederManager.setFeederState(FeederState.IDLE);
    //                         hopperManager.setHopperState(HopperState.IDLE);
    //                         intakeManager.setIntakeState(IntakeState.DEPLOYED);
    //                     }));
    // }

    public Command getAutoCommand() {

        path_test.setPathConstraints(constraints);
        return Commands.sequence(
                        new InstantCommand(() -> {
                            Pose2d startPose = path_test.getStartPose();
                            if (Util.isRedSide()) {
                                startPose = FlippingUtil.flipFieldPose(startPose);
                            }
                            // drive.resetOdometry(startPose);
                        }),
                        drive.followPathWithHubRotation(path_test, 1.5).withName("Path 2"))

                // Commands.deadline(Commands.waitSeconds(4), getAutoRotateCommand()),
                // drive.getPathBuilder().build(path_3).withName("climb"))
                .withName("Entire sequence");
    }
}
