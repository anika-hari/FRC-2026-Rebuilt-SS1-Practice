// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.config.ConfigConstants;
import frc.config.RobotConfig.Subsystem;
import frc.lib.BLine.FollowPath;
import frc.lib.subsystems.FlywheelIO;
import frc.lib.subsystems.IOFactory;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.canDevice.CanCoderIO;
import frc.lib.subsystems.real.FlywheelSubsystemConfig;
import frc.lib.subsystems.real.ServoMotorSubsystemConfig;
import frc.lib.subsystems.real.ServoMotorSubsystemWithCanCoderConfig;
import frc.lib.subsystems.real.ServoMotorSubsystemWithFollowersConfig;
import frc.lib.util.Controls.StreamDeck.StreamDeck;
import frc.robot.autos.DCMPPlayoffsAuto;
import frc.robot.autos.DCMPPlayoffsAutoMirrored;
import frc.robot.autos.DoubleSwipeAgainstHub;
import frc.robot.autos.DoubleSwipeAgainstHubMirrored;
import frc.robot.autos.DoubleSwipeAggro;
import frc.robot.autos.DoubleSwipeAggroMirrored;
import frc.robot.autos.DoubleSwipeCenter;
import frc.robot.autos.DoubleSwipeCenterMirrored;
import frc.robot.autos.DoubleSwipeCross;
import frc.robot.autos.DoubleSwipeCrossMirrored;
import frc.robot.autos.DoubleSwipeDoubleAggro;
import frc.robot.autos.DoubleSwipeDoubleAggroMirrored;
import frc.robot.autos.DoubleSwipeFar;
import frc.robot.autos.DoubleSwipeMirrored;
import frc.robot.autos.DoubleSwipeNear;
import frc.robot.autos.DoubleSwipeNearMirrored;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.hopper.HopperConstants.HopperState;
import frc.robot.subsystems.hopper.HopperManager;
import frc.robot.viz.RobotVisualization;
import java.util.Arrays;

public class RobotContainer {


    private final Hopper hopper;
    private final HopperManager hopperManager;


    private final RobotState robotState;

    private final CommandXboxController controller = new CommandXboxController(0);
    private final StreamDeck streamdeck = new StreamDeck();
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        robotState = new RobotState();


        hopper = buildHopper();


        hopperManager = buildHopperManager(this);

        System.out.print(ConfigConstants.robotIdentity);
    }


    /** Helper method to check if a subsystem is enabled in the current robot configuration. */
    @SuppressWarnings("unused")
    private boolean isEnabled(Subsystem subsystem) {
        return ConfigConstants.ROBOT_CONFIG.isEnabled(subsystem);
    }

    /** Helper method to check if a subsystem is disabled in the current robot configuration. */
    private boolean isDisabled(Subsystem subsystem) {
        return ConfigConstants.ROBOT_CONFIG.isDisabled(subsystem);
    }


   

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

  
    public Hopper getHopper() {
        return hopper;
    }

   

    public HopperManager getHopperManager() {
        return hopperManager;
    }

    public CommandXboxController getController() {
        return controller;
    }

    public StreamDeck getStreamDeck() {
        return streamdeck;
    }

 

    private Hopper buildHopper() {
        ServoMotorSubsystemConfig config = HopperConstants.kHopperConfig;
        boolean disabled = isDisabled(Subsystem.HOPPER);
        MotorIO io = IOFactory.motor(config, disabled);
        return new Hopper(config, io);
    }

  

  

    private HopperManager buildHopperManager(RobotContainer container) {
        return new HopperManager(container);
    }


}
