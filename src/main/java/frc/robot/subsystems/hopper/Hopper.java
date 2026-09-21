package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.real.ServoMotorSubsystem;
import frc.lib.subsystems.real.ServoMotorSubsystemConfig;
import frc.lib.util.LoggedTunableNumber;

public class Hopper extends ServoMotorSubsystem<MotorIO> {

    //Create a tunable number for the voltage the hopper should run at
    private final LoggedTunableNumber hopperVoltageTunable = new LoggedTunableNumber("Hopper/Hopper Voltage", 0.5);

    //Create a constructor for the hopper class that takes in a ServoMotorSubsystemConfig and MotorIO. Call the super constructor.
    public Hopper(ServoMotorSubsystemConfig config, MotorIO motorIO) {
        super(config, motorIO);
    }


    //Create a periodic function that overrides the ServoMotorSubsystem periodic. Call the super for the method.
    @Override 
    public void periodic() {
        super.periodic();
    }

    
    //Create 3 funtions that stops the hopper, reverses the hopper, and runs the hopper
    public void hopperStop(){
        setVoltageImpl(0);
    }
    public void hopperForwardVoltage(){
        setVoltageImpl(hopperVoltageTunable.get());
    }
    public void hopperReverseVoltage(){
        setVoltageImpl(-hopperVoltageTunable.get());
    }
    //Create 3 commands that stop, start, and reverse the hopper
    public Command Stop(){
        return Commands.runOnce(
            () -> hopperStop()
        );
    }
    public Command Start(){
        return Commands.runOnce(
            () -> hopperForwardVoltage()
        );
    }
    public Command Reverse(){
        return Commands.runOnce(
            () -> hopperReverseVoltage()
        );
    }
 

    @Override
    public LoggedTunableNumber[] getTunables() {
        return new LoggedTunableNumber[] {hopperVoltageTunable};
    }
    
}
