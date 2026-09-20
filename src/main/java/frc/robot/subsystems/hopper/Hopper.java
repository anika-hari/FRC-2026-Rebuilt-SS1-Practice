package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.real.ServoMotorSubsystem;
import frc.lib.subsystems.real.ServoMotorSubsystemConfig;
import frc.lib.util.LoggedTunableNumber;

public class Hopper extends ServoMotorSubsystem<MotorIO> {

    //Create a tunable number for the voltage the hopper should run at

    //Create a constructor for the hopper class that takes in a ServoMotorSubsystemConfig and MotorIO. Call the super constructor.


    //Create a periodic function that overrides the ServoMotorSubsystem periodic. Call the super for the method.


    
    //Create 3 funtions that stops the hopper, reverses the hopper, and runs the hopper


    //Create 3 commands that stop, start, and reverse the hopper

 

    @Override
    public LoggedTunableNumber[] getTunables() {
        return new LoggedTunableNumber[] {hopperVoltageTunable};
    }
}
