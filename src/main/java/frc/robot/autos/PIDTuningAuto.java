package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.BLine.Path;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;

public class PIDTuningAuto {
    private Drive drive;

    public PIDTuningAuto(RobotContainer container) {
        this.drive = container.getDrive();
    }

    Path myPath = new Path("TranslationPIDTuning");

    public Command getAutoCommand() {
        return Commands.sequence(drive.getPathBuilder().build(myPath));
    }
}
