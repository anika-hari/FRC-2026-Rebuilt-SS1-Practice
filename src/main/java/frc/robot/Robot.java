// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.config.ConfigConstants;
import frc.lib.subsystems.CTREStatusSignalManager;
import frc.lib.util.BatteryLogger;
import frc.lib.util.LoopTimingLogger;
import frc.lib.util.MacAddressUtil;
import frc.lib.util.MagicVirtualSubsystem;
import java.io.File;
import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.file.Files;
import java.util.Arrays;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private static final int kRTPriority = 2;
    private static final int kNonRTPriority = 1;
    private static final double loopOverrunWarningTimeout = 1;

    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    // Staged NT initialization to avoid cold start topic storm
    private int startupLoopCount = 0;
    private boolean elasticInitialized = false;
    private boolean streamDeckInitialized = false;
    private static final int ELASTIC_INIT_DELAY_LOOPS = Constants.elasticButtonsEnabled ? 75 : 0; // ~1.5s at 50Hz
    private static final int STREAMDECK_INIT_DELAY_LOOPS = ELASTIC_INIT_DELAY_LOOPS + 75; // ~3.0s at 50Hz

    public Robot() {
        Logger.recordMetadata("CO 2026", "MyProject"); // Set a metadata value
        m_robotContainer = new RobotContainer();

        // 6 V is low, but safe. 1678 has set this as low as 4.6.
        RobotController.setBrownoutVoltage(6);

        switch (Constants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                String LOG_DIRECTORY = "/home/lvuser/logs";
                long MIN_FREE_SPACE = 1000000000;
                var directory = new File(LOG_DIRECTORY);
                if (!directory.exists()) {
                    directory.mkdir();
                }

                // ensure that there is enough space on the roboRIO to log data
                if (directory.getFreeSpace() < MIN_FREE_SPACE) {
                    var files = directory.listFiles();
                    if (files != null) {
                        // Sorting the files by name will ensure that the oldest files are deleted first
                        files = Arrays.stream(files).sorted().toArray(File[]::new);

                        long bytesToDelete = MIN_FREE_SPACE - directory.getFreeSpace();

                        for (File file : files) {
                            if (file.getName().endsWith(".wpilog")) {
                                try {
                                    bytesToDelete -= Files.size(file.toPath());
                                } catch (IOException e) {
                                    System.out.println("Failed to get size of file " + file.getName());
                                    continue;
                                }
                                if (file.delete()) {
                                    System.out.println("Deleted " + file.getName() + " to free up space");
                                } else {
                                    System.out.println("Failed to delete " + file.getName());
                                }
                                if (bytesToDelete <= 0) {
                                    break;
                                }
                            }
                        }
                    }
                }
                Logger.addDataReceiver(new WPILOGWriter(LOG_DIRECTORY));
                if (!DriverStation.isFMSAttached()) {
                    Logger.addDataReceiver(new RLOGServer());
                }
                break;

            case SIM:
                // Disabled RLog for sim since Thomas is on Linux. The Sim GUI Doesn't have RLog and Ascope is very
                // laggy on Linux
                // Running a physics simulator, log to NT
                // Logger.addDataReceiver(new RLOGServer());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // Start AdvantageKit logger
        Logger.start();

        // Disable LiveWindow telemetry to reduce framework overhead in loopFunc().
        // LiveWindow registers all SubsystemBase instances as Sendable objects and
        // updates them every loop via SmartDashboard.updateValues(), which can cause
        // periodic 50-150ms spikes.
        LiveWindow.disableAllTelemetry();

        Logger.recordOutput("Robot/Robot MAC", MacAddressUtil.macToString(ConfigConstants.mac));
        Logger.recordOutput("Robot/Robot Identity", ConfigConstants.robotIdentity);

        // Silence Joystick Connection Warnings
        DriverStation.silenceJoystickConnectionWarning(true);

        // Silence Loop Time Overrun Warnings
        try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(loopOverrunWarningTimeout);
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
        }
        CommandScheduler.getInstance().setPeriod(loopOverrunWarningTimeout);

        // Silence Rotation2d warnings
        var mathShared = MathSharedStore.getMathShared();
        MathSharedStore.setMathShared(new MathShared() {
            @Override
            public void reportError(String error, StackTraceElement[] stackTrace) {
                if (error.startsWith("x and y components of Rotation2d are zero")) {
                    return;
                }
                mathShared.reportError(error, stackTrace);
            }

            @Override
            public void reportUsage(MathUsageId id, int count) {
                mathShared.reportUsage(id, count);
            }

            @Override
            public double getTimestamp() {
                return mathShared.getTimestamp();
            }
        });
        SmartDashboard.putBoolean("Drive/isXLocked", false);
    }

    @Override
    protected void loopFunc() {
        LoopTimingLogger.startTiming("FullLoopFunc");
        super.loopFunc();
        LoopTimingLogger.endTiming("FullLoopFunc");
    }

    @Override
    public void robotPeriodic() {

        // Staged NT initialization: defer dashboard topics to avoid cold start topic storm.
        // Stage 1 (core robot + Logger) runs immediately in the constructor.
        // Stage 2 (Elastic) and Stage 3 (StreamDeck) are deferred here.
        if (!streamDeckInitialized) {
            startupLoopCount++;
            if (!elasticInitialized && startupLoopCount >= ELASTIC_INIT_DELAY_LOOPS) {
                if (Constants.elasticButtonsEnabled) {
                    m_robotContainer.initializeElasticDashboard();
                    System.out.println("[STARTUP] Elastic dashboard initialized at loop " + startupLoopCount);
                }
                elasticInitialized = true;
            }
            if (startupLoopCount >= STREAMDECK_INIT_DELAY_LOOPS) {
                m_robotContainer.initializeStreamDeck();
                System.out.println("[STARTUP] StreamDeck initialized at loop " + startupLoopCount);
                streamDeckInitialized = true;
            }
        }

        // Start timing measurement for the entire robotPeriodic method
        LoopTimingLogger.startTiming("RobotPeriodic");

        // Elevate priority when enabled for responsive control
        if (DriverStation.isEnabled()) {
            Threads.setCurrentThreadPriority(true, kRTPriority);
        } else {
            Threads.setCurrentThreadPriority(false, kNonRTPriority);
        }
        // Threads.setCurrentThreadPriority(true, 99);

        LoopTimingLogger.startTiming("HubStateTracker");
        HubStateTracker.updateGameData();
        SmartDashboard.putNumber(
                "Robot/MatchTimer",
                Math.max(
                        Math.ceil(HubStateTracker.getUntilNextInactive().orElse(0.0) * 10) / 10,
                        Math.ceil(HubStateTracker.getUntilNextActive().orElse(0.0) * 10) / 10));
        LoopTimingLogger.endTiming("HubStateTracker");
        // Refresh all registered MotorIO signals once per loop and populate the
        // corresponding MotorInputs objects before commands and subsystem periodics run.
        LoopTimingLogger.startTiming("CTREStatusSignalManager");
        CTREStatusSignalManager.refreshAndReadAll();
        LoopTimingLogger.endTiming("CTREStatusSignalManager");
        LoopTimingLogger.startTiming("MagicVirtualSubsystemPeriodic");
        MagicVirtualSubsystem.runPeriodically();
        LoopTimingLogger.endTiming("MagicVirtualSubsystemPeriodic");
        LoopTimingLogger.startTiming("CommandScheduler");
        CommandScheduler.getInstance().run();
        LoopTimingLogger.endTiming("CommandScheduler");

        LoopTimingLogger.startTiming("BatteryLogger");
        BatteryLogger.periodicAfterScheduler();
        LoopTimingLogger.endTiming("BatteryLogger");

        // Reset priority at end of loop to yield CPU to background tasks
        Threads.setCurrentThreadPriority(false, kNonRTPriority);
        LoopTimingLogger.endTiming("RobotPeriodic");
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        HubStateTracker.gameTimer.start();
    }

    @Override
    public void autonomousPeriodic() {
        LoopTimingLogger.startTiming("AutonomousPeriodic");
        LoopTimingLogger.endTiming("AutonomousPeriodic");
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        HubStateTracker.gameTimer.reset();
        HubStateTracker.gameTimer.start();
        HubStateTracker.doneWithAuto = true;
    }

    @Override
    public void teleopPeriodic() {
        LoopTimingLogger.startTiming("TeleopPeriodic");
        LoopTimingLogger.endTiming("TeleopPeriodic");
    }

    @Override
    public void teleopExit() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {
        LoopTimingLogger.startTiming("SimulationPeriodic");
        LoopTimingLogger.startTiming("MagicVirtualSubsystemSimulation");
        MagicVirtualSubsystem.runSimulationPeriodically();
        LoopTimingLogger.endTiming("MagicVirtualSubsystemSimulation");
        LoopTimingLogger.startTiming("VizUpdate");
        m_robotContainer.getVisualization().updateViz();
        LoopTimingLogger.endTiming("VizUpdate");
        LoopTimingLogger.endTiming("SimulationPeriodic");
    }

    @Override
    public void testInit() {
   
    }

    @Override
    public void testPeriodic() {
        LoopTimingLogger.startTiming("TestPeriodic");
        LoopTimingLogger.endTiming("TestPeriodic");
    }

    @Override
    public void testExit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
