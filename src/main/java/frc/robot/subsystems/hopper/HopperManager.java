package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.MagicVirtualSubsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hopper.HopperConstants.HopperState;
import org.littletonrobotics.junction.Logger;

public class HopperManager extends MagicVirtualSubsystem {
    //Define a hopper variable and hopperState variable.
    private Hopper hopper;
    private HopperState hopperState = HopperState.IDLE;

    //Create a constructor that takes in a RobotContainer. inside the constructor, set the hopper variable to the Hopper object from RobotContainer
    // Mark manager as disabled if subsystem are disabled
    //Set the default command for the hopper to the hopperDefault command you will make below.

    //Create a periodic that logs the hopper state
   
    //Create an empty SimulationPeriodic

    //Create a hopperDefault function taht returns a command
    //Use a run command to check the hopperState, and runs an action depending on the state


    //Create 2 functions to set and get the hopper state


    //create 3 functions that return commands that run, reverse, and stop the hopper but setting the hopper state

}
