package frc.robot.autos;

import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;

public abstract class BaseAuto {
    protected Drive drive;

    public BaseAuto(RobotContainer container) {
        this.drive = container.getDrive();
    }
}
