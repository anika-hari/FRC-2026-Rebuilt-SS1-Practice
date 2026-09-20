package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BooleanSupplier;

public class EitherUpdateCommand extends Command {

    private Runnable onTrue;
    private Runnable onFalse;
    private BooleanSupplier condition;

    public EitherUpdateCommand(Runnable onTrue, Runnable onFalse, BooleanSupplier condition, Subsystem... require) {
        this.onTrue = onTrue;
        this.onFalse = onFalse;
        this.condition = condition;
        this.addRequirements(require);
    }

    @Override
    public void execute() {
        if (condition.getAsBoolean()) {
            onTrue.run();
        } else {
            onFalse.run();
        }
    }
}
