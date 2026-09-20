package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.RobotState;
import frc.robot.game_util.FieldConstants;
import frc.robot.subsystems.hopper.HopperConstants.HopperState;
import frc.robot.subsystems.hopper.HopperManager;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterManager;
import frc.robot.subsystems.shooter.ShooterManager.ShooterOverride;
import org.littletonrobotics.junction.Logger;

public class ShooterTuningCommand extends Command {

    private final Shooter shooter;
    private final Hood hood;
    private final ShooterManager shooterManager;
    private final HopperManager hopperManager;

    private final LoggedTunableNumber tuningRPM = new LoggedTunableNumber("ShooterTuning/RPM", 3000);
    private final LoggedTunableNumber tuningHoodDeg = new LoggedTunableNumber("ShooterTuning/HoodAngleDeg", 60);

    public ShooterTuningCommand(
            Shooter shooter, Hood hood, ShooterManager shooterManager, HopperManager hopperManager) {
        this.shooter = shooter;
        this.hood = hood;
        this.shooterManager = shooterManager;
        this.hopperManager = hopperManager;
        addRequirements(shooter, hood);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double distance = 0;
        if (DriverStation.getAlliance().isPresent()) {
            Translation2d robotTranslation = RobotState.getGlobalPose().getTranslation();
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                distance = FieldConstants.Hub.blueHubCenter2d.getDistance(robotTranslation);
            } else {
                distance = FieldConstants.Hub.redHubCenter2d.getDistance(robotTranslation);
            }
        }

        double rpmValue = tuningRPM.getAsDouble();
        double hoodDegValue = tuningHoodDeg.getAsDouble();

        shooter.setShooterCurrentFOCVelocity(() -> Units.rotationsPerMinuteToRadiansPerSecond(rpmValue));
        hood.setHoodAngle(Units.degreesToRadians(hoodDegValue));

        if (shooter.isAtSetpoint() && hood.isAtSetpoint()) {
            shooterManager.setShooterOverride(ShooterOverride.TUNING);
            hopperManager.setHopperState(HopperState.FEEDING);
            shooter.setShooting(true);
        }

        Logger.recordOutput("ShooterTuning/DistanceToHub", distance);
        Logger.recordOutput("ShooterTuning/DistanceToHubInches", Units.metersToInches(distance));
        Logger.recordOutput("ShooterTuning/RPM", rpmValue);
        Logger.recordOutput("ShooterTuning/HoodAngleDeg", hoodDegValue);
        Logger.recordOutput("ShooterTuning/FlywheelAtSetpoint", shooter.isAtSetpoint());
        Logger.recordOutput("ShooterTuning/HoodAtSetpoint", hood.isAtSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        shooterManager.setShooterOverride(ShooterOverride.NONE);
        hopperManager.setHopperState(HopperState.IDLE);
        shooter.setShooting(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
