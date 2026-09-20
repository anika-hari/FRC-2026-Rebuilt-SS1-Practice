package frc.robot.game_util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import frc.robot.game_util.FieldConstants.AprilTagStruct;
import frc.robot.RobotState;

public class FieldUtils {
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    }

    public static boolean isBlueAlliance() {
        return FieldUtils.getAlliance() == Alliance.Blue;
    }

    public static boolean isRedAlliance() {
        return FieldUtils.getAlliance() == Alliance.Red;
    }

    public static double getFlipped() {
        return FieldUtils.isRedAlliance() ? -1 : 1;
    }

    // Field Constants don't have apriltagstruct yet
    // public static AprilTagStruct getClosestHPSTag() {
    //   List<AprilTagStruct> hpsTags =
    //       FieldUtils.isBlueAlliance() ? FieldConstants.blueHPSTags : FieldConstants.redHPSTags;

    //   Translation2d robotTranslation = RobotState.getGlobalPose().getTranslation();

    //   AprilTagStruct closestTag =
    //       hpsTags.stream()
    //           .reduce(
    //               (AprilTagStruct tag1, AprilTagStruct tag2) ->
    //                   robotTranslation.getDistance(tag1.pose().getTranslation().toTranslation2d())
    //                           < robotTranslation.getDistance(
    //                               tag2.pose().getTranslation().toTranslation2d())
    //                       ? tag1
    //                       : tag2)
    //           .get();

    //   return closestTag;
    // }

    // TODO: Confirm blue/red sides are correct
    public static boolean isOnAllianceSide() {
        double robotX = RobotState.getGlobalPose().getTranslation().getX();
        if (FieldUtils.isBlueAlliance()) {
            return robotX < FieldConstants.fieldLengthMeters / 2;
        } else {
            return robotX > FieldConstants.fieldLengthMeters / 2;
        }
    }

    public static boolean isOnRedSide() {
        double robotX = RobotState.getGlobalPose().getTranslation().getX();
        return robotX > FieldConstants.fieldLengthMeters / 2;
    }

    public static boolean isOnBlueSide() {
        double robotX = RobotState.getGlobalPose().getTranslation().getX();
        return robotX < FieldConstants.fieldLengthMeters / 2;
    }

    public static boolean isInsideField(Pose2d pose) {
        return isInsideField(pose, 0.0);
    }

    public static boolean isInsideField(Pose2d pose, double bufferMeters) {
        return pose.getX() >= bufferMeters
                && pose.getX() <= FieldConstants.fieldLengthMeters - bufferMeters
                && pose.getY() >= bufferMeters
                && pose.getY() <= FieldConstants.fieldWidthMeters - bufferMeters;
    }
}
