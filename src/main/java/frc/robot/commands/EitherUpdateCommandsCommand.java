package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class EitherUpdateCommandsCommand extends Command {
    private Command onTrue;
    private Command onFalse;
    private boolean onTrueScheduled;
    private boolean onFalseScheduled;

    private BooleanSupplier condition;

    public EitherUpdateCommandsCommand(
            Command onTrue, Command onFalse, BooleanSupplier condition, Subsystem... require) {
        this.onTrue = onTrue;
        this.onFalse = onFalse;
        onTrueScheduled = false;
        onFalseScheduled = false;
        this.condition = condition;
        this.addRequirements(require);
    }

    public void initialize() {
        onTrueScheduled = false;
        onFalseScheduled = false;
        if (condition.getAsBoolean()) {
            CommandScheduler.getInstance().schedule(onTrue);
            onTrueScheduled = true;
        } else {
            CommandScheduler.getInstance().schedule(onFalse);
            onFalseScheduled = true;
        }
    }

    @Override
    public void execute() {
        Logger.recordOutput("EitherUpdateCommand" + "/Condition", condition.getAsBoolean());
        Logger.recordOutput("EitherUpdateCommand" + "/onTrueScheduled", onTrueScheduled);
        Logger.recordOutput("EitherUpdateCommand" + "/onFalseScheduled", onFalseScheduled);
        if (condition.getAsBoolean() && !onTrueScheduled) {
            CommandScheduler.getInstance().schedule(onTrue);
            onTrueScheduled = true;
            onFalseScheduled = false;
        } else if (!condition.getAsBoolean() && !onFalseScheduled) {
            CommandScheduler.getInstance().schedule(onFalse);
            onTrueScheduled = false;
            onFalseScheduled = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (onTrueScheduled) {
            onTrue.cancel();
        }
        if (onFalseScheduled) {
            onFalse.cancel();
        }
        onTrueScheduled = false;
        onFalseScheduled = false;
    }
}
