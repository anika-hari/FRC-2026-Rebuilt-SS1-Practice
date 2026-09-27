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
    public HopperManager(RobotContainer robotContainer){
        this.hopper=robotContainer.getHopper();
        if (hopper.isDisabled())
        setDisabled(true);
    
        hopper.setDefaultCommand(hopperDefault());
    }
    // Mark manager as disabled if subsystem are disabled
    //Set the default command for the hopper to the hopperDefault command you will make below.

    //Create a periodic that logs the hopper state
    @Override 
    public void periodic(){
        Logger.recordOutput("Hopper/HopperState", hopperState);
    }

    //Create an empty SimulationPeriodic
    @Override
    public void simulationPeriodic(){
        
    }

    //Create a hopperDefault function taht returns a command
    public Command hopperDefault() {
        return Commands.run(
        ()-> {
            switch (hopperState){
                case IDLE:
                    hopper.hopperStop();
                case FEEDING:
                    hopper.Start();
                 
                case REVERSE:
                    hopper.Reverse();
                case JAMMED:
                    hopper.hopperStop();
            }
        }
        );
    }
    //Use a run command to check the hopperState, and runs an action depending on the state


    //Create 2 functions to set and get the hopper state
    public void setHopperState(HopperState state){
        hopperState = state;

    }
    public HopperState getHopperState(){
       return hopperState;
    }

    //create 3 functions that return commands that run, reverse, and stop the hopper but setting the hopper state
    public void setHopperIDLE(){
        hopperState = HopperState.IDLE;
    }
     public void setHopperJAMMED(){
        hopperState = HopperState.JAMMED;
    }
     public void setHopperFEEDING(){
        hopperState = HopperState.FEEDING;
    }
     public void setHopperREVERSE(){
        hopperState = HopperState.IDLE;
    }
}
