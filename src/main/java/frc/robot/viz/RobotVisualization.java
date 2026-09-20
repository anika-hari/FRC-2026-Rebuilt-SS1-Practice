package frc.robot.viz;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.util.COColor;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class RobotVisualization {

    private RobotContainer robotContainer;

    private LoggedMechanism2d mechanism2d = new LoggedMechanism2d(2, 3, COColor.kGray.getColor8Bit());

    private LoggedMechanismRoot2d driveRoot2d;
    private LoggedMechanismLigament2d driveLigament;

    private LoggedMechanismRoot2d intakeRoot2d;
    private LoggedMechanismLigament2d intakeArmLigament;
    private LoggedMechanismLigament2d intakeRollerLigament;

    private LoggedMechanismRoot2d hopperHotdogRoller1Root2d;
    private LoggedMechanismRoot2d hopperHotdogRoller2Root2d;
    private LoggedMechanismLigament2d hopperHotdogRoller1ligament;
    private LoggedMechanismLigament2d hopperHotdogRoller2ligament;

    private LoggedMechanismRoot2d shooterRoot2d;
    private LoggedMechanismLigament2d shooterWheelLigament;
    private LoggedMechanismLigament2d shooterHoodLigament;

    private LoggedMechanismRoot2d boxRoot2d;
    private LoggedMechanismLigament2d boxLigament;

    private LoggedMechanismRoot2d feederTopRoot2d;
    private LoggedMechanismRoot2d feederBotRoot2d;
    private LoggedMechanismLigament2d feederTopLigament;
    private LoggedMechanismLigament2d feederBotLigament;

    private LoggedMechanismRoot2d ledLeftRoot2d;
    private LoggedMechanismLigament2d ledLeftLigament2d;
    private LoggedMechanismRoot2d ledRightRoot2d;
    private LoggedMechanismLigament2d ledRightLigament2d;

    public RobotVisualization(RobotContainer container) {
        this.robotContainer = container;
        if (Robot.isReal()) return;
        initViz();
    }

    public void initViz() {
        // Drive
        driveRoot2d = mechanism2d.getRoot("driveRoot", VizConstants.drivebase_x, VizConstants.drivebase_top_y);
        driveLigament = new LoggedMechanismLigament2d(
                "drive", Units.inchesToMeters(DriveConstants.kBumperLengthInches), 0, 5, new Color8Bit(0, 208, 255));
        driveRoot2d.append(driveLigament);
        // Intake
        intakeRoot2d = mechanism2d.getRoot("intakeRoot", VizConstants.intake_x, VizConstants.intake_y);
        intakeArmLigament = new LoggedMechanismLigament2d("intakeArm", 0.5, 0, 5, new Color8Bit(0, 208, 255));
        intakeRollerLigament = new LoggedMechanismLigament2d("intakeRoller", 0.1, 0, 5, new Color8Bit(255, 0, 0));
        intakeRoot2d.append(intakeArmLigament);
        intakeArmLigament.append(intakeRollerLigament);

        // Box
        boxRoot2d = mechanism2d.getRoot("boxRoot", VizConstants.box_x, VizConstants.box_y);
        boxLigament = new LoggedMechanismLigament2d("boxElevator", 0.0, 90, 5, new Color8Bit(255, 0, 0));
        boxRoot2d.append(boxLigament);
        // Hopper
        hopperHotdogRoller1Root2d = mechanism2d.getRoot(
                "hopperHotDogRoller1Root", VizConstants.hopper_hotdog_1_x, VizConstants.hopper_hotdogs_y);
        hopperHotdogRoller1ligament =
                new LoggedMechanismLigament2d("hopperHotdogRoller1", 0.03, 0, 5, new Color8Bit(255, 0, 0));
        hopperHotdogRoller1Root2d.append(hopperHotdogRoller1ligament);
        hopperHotdogRoller2Root2d = mechanism2d.getRoot(
                "hopperHotDogRoller2Root", VizConstants.hopper_hotdog_2_x, VizConstants.hopper_hotdogs_y);
        hopperHotdogRoller2ligament =
                new LoggedMechanismLigament2d("hopperHotdogRoller2", 0.03, 0, 5, new Color8Bit(255, 0, 0));
        hopperHotdogRoller2Root2d.append(hopperHotdogRoller2ligament);
        // Shooter
        shooterRoot2d = mechanism2d.getRoot("shooterRoot", VizConstants.shooter_x, VizConstants.shooter_y);
        shooterHoodLigament = new LoggedMechanismLigament2d("shooterHood", 0.3, 0, 3, new Color8Bit(0, 255, 0));
        shooterWheelLigament = new LoggedMechanismLigament2d("shooterWheel", 0.03, 0, 5, new Color8Bit(255, 0, 0));
        shooterRoot2d.append(shooterHoodLigament);
        shooterRoot2d.append(shooterWheelLigament);
        // Led
        ledLeftRoot2d = mechanism2d.getRoot("leftLedRoot", VizConstants.left_led_x, VizConstants.led_y);
        ledLeftLigament2d = new LoggedMechanismLigament2d("leftLed", 0.4, 90, 4, new Color8Bit(255, 255, 255));
        ledLeftRoot2d.append(ledLeftLigament2d);
        ledRightRoot2d = mechanism2d.getRoot("rightLedRoot", VizConstants.right_led_x, VizConstants.led_y);
        ledRightLigament2d = new LoggedMechanismLigament2d("rightLed", 0.4, 90, 4, new Color8Bit(255, 255, 255));
        ledRightRoot2d.append(ledRightLigament2d);

        // feeder
        feederBotRoot2d = mechanism2d.getRoot("feederBotRoot", VizConstants.feeder_x, VizConstants.feederbot_y);
        feederBotLigament = new LoggedMechanismLigament2d("feederBotRoller", 0.05, 0, 5, new Color8Bit(255, 0, 0));
        feederBotRoot2d.append(feederBotLigament);

        feederTopRoot2d = mechanism2d.getRoot("feederTopRoot", VizConstants.feeder_x, VizConstants.feedertop_y);
        feederTopLigament = new LoggedMechanismLigament2d("feederTopRoller", 0.05, 0, 5, new Color8Bit(255, 0, 0));
        feederTopRoot2d.append(feederTopLigament);
    }

    public void updateViz() {
        double intakeArmRad = robotContainer.getIntakePivot().getPosition();
        double intakeRollerRad = robotContainer.getIntakeRoller().getPosition();
        double hopperRollerRad = robotContainer.getHopper().getPosition();
        intakeArmLigament.setAngle(Units.radiansToDegrees(intakeArmRad));
        intakeRollerLigament.setAngle(Units.radiansToDegrees(intakeRollerRad));
        hopperHotdogRoller1ligament.setAngle(-Units.radiansToDegrees(hopperRollerRad));
        hopperHotdogRoller2ligament.setAngle(-Units.radiansToDegrees(hopperRollerRad));

        double shooterFlywheelRad = robotContainer.getShooter().getPosition();
        double shooterHoodRad = robotContainer.getHood().getPosition() + Units.degreesToRadians(90);
        shooterWheelLigament.setAngle(Units.radiansToDegrees(shooterFlywheelRad));
        shooterHoodLigament.setAngle(Units.radiansToDegrees(shooterHoodRad));

        double boxHeightMeters = robotContainer.getBox().getPosition();
        boxLigament.setLength(boxHeightMeters);

        double feederTopRad = robotContainer.getShooter().getPosition();
        double feederBotRad = robotContainer.getShooter().getPosition();
        feederTopLigament.setAngle(Units.radiansToDegrees(feederTopRad));
        feederBotLigament.setAngle(Units.radiansToDegrees(feederBotRad));

        ledLeftLigament2d.setColor(RobotState.getLeftLedState().getPureColor8Bit());
        ledRightLigament2d.setColor(RobotState.getRightLedState().getPureColor8Bit());
        Logger.recordOutput("SimViz", mechanism2d);
    }
}
