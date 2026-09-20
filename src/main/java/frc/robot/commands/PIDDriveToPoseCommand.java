package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PIDDriveToPoseCommand extends Command {
    private final ProfiledPIDController distanceController = new ProfiledPIDController(
            DriveConstants.DRIVE_TO_POSE_KP,
            DriveConstants.DRIVE_TO_POSE_KI,
            DriveConstants.DRIVE_TO_POSE_KD,
            new TrapezoidProfile.Constraints(
                    (DriveConstants.kDriveMaxSpeed / 2), DriveConstants.kMaxAccelerationMetersPerSecondSquared / 2));
    private final ProfiledPIDController angleController = new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                    DriveConstants.kDriveMaxAngularRate, DriveConstants.ANGLE_MAX_ACCELERATION));

    private final Drive drive;
    private final Supplier<Pose2d> targetPoseSupplier;

    private double ffMinRadius = 0.0, ffMaxRadius = 0.1;

    private double maxVelocity = DriveConstants.kDriveMaxSpeed;

    private final ApplyRobotSpeeds robotSpeeds = new ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withDesaturateWheelSpeeds(true);

    public PIDDriveToPoseCommand(RobotContainer container, Supplier<Pose2d> targetPoseSupplier) {
        drive = container.getDrive();
        addRequirements(drive);
        this.targetPoseSupplier = targetPoseSupplier;
        distanceController.setTolerance(Units.inchesToMeters(0.7));
        angleController.setTolerance(Units.degreesToRadians(1.5));
    }

    public Trigger atSetpoint() {
        return new Trigger(() -> distanceController.atSetpoint() && angleController.atSetpoint());
    }

    public PIDDriveToPoseCommand withJoystickRumble(Command rumbleCommand) {
        atSetpoint().onTrue(Commands.deferredProxy(() -> rumbleCommand));

        return this;
    }

    @Override
    public void initialize() {}

    // Heavily inspired by 6328's `AutoAlignController` from 2024
    // https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/drive/controllers/AutoAlignController.java#L135
    @Override
    public void execute() {

        Pose2d robot = RobotState.getGlobalPose();
        Pose2d target = targetPoseSupplier.get();

        double currentDistance = robot.getTranslation().getDistance(target.getTranslation());
        double ffScaler = MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);

        Translation2d translationalError = robot.getTranslation().minus(target.getTranslation());
        Rotation2d angularError = robot.getRotation().minus(target.getRotation());

        // Magnitude of translational velocity, meaning that x & y are controlled together
        double translationalSpeed = distanceController.getSetpoint().velocity * ffScaler
                + MathUtil.clamp(
                        distanceController.calculate(translationalError.getNorm(), 0), -maxVelocity, maxVelocity);
        translationalSpeed = !distanceController.atGoal() ? translationalSpeed : 0;

        double angularVelocity = angleController.getSetpoint().velocity * ffScaler
                + angleController.calculate(angularError.getRadians(), 0);
        angularVelocity = !angleController.atGoal() ? angularVelocity : 0;

        Translation2d velocity = new Translation2d(translationalSpeed, translationalError.getAngle());

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                velocity.getX(), velocity.getY(), angularVelocity, robot.getRotation());

        drive.setControl(robotSpeeds.withSpeeds(speeds));

        Logger.recordOutput("Commands/" + getName() + "/Robot", robot);
        Logger.recordOutput("Commands/" + getName() + "/Target", target);
        Logger.recordOutput("Commands/" + getName() + "/Error/Translational", translationalError);
        Logger.recordOutput("Commands/" + getName() + "/Error/Angular", angularError);
        Logger.recordOutput("Commands/" + getName() + "/AtGoal/Translation", distanceController.atGoal());
        Logger.recordOutput("Commands/" + getName() + "/AtGoal/Angle", angleController.atGoal());
        Logger.recordOutput("Commands/" + getName() + "/Speeds/Translational", translationalSpeed);
        Logger.recordOutput("Commands/" + getName() + "/Speeds/ChassisSpeeds", speeds);
    }

    @Override
    public boolean isFinished() {
        return angleController.atGoal() && distanceController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        drive.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds()));
        distanceController.reset(0);
        angleController.reset(0);
    }
}
