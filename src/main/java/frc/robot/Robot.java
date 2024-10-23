// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import java.util.Map;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.northernforce.util.NFRRobotChooser;
import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.robots.CrabbyContainer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot
{
    private Command autoSelected = null;
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    private NFRRobotContainer container = null;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit()
    {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY)
        {
        case 0:
            Logger.recordMetadata("GitDirty", "All changes committed");
            break;
        case 1:
            Logger.recordMetadata("GitDirty", "Uncomitted changes");
            break;
        default:
            Logger.recordMetadata("GitDirty", "Unknown");
            break;
        }

        final NFRRobotChooser chooser = new NFRRobotChooser(() -> new CrabbyContainer(),
                Map.of("Crabby", () -> new CrabbyContainer()));
        container = chooser.getNFRRobotContainer();

        // Set up data receivers & replay source
        switch (Constants.kCurrentMode)
        {
        case REAL:
            // Running on a real robot, log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
            break;

        case SIM:
            // Running a physics simulator, log to NT
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

        // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
        // Logger.disableDeterministicTimestamps()

        // Start AdvantageKit logger
        Logger.start();

        // Initialize auto chooser
        final var defaultAuto = container.getDefaultAutonomous();
        autoChooser.addDefaultOption(defaultAuto.getFirst(), defaultAuto.getSecond());
        container.getAutonomousOptions().forEach(autoChooser::addOption);

    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
        container.periodic();
    }

    /** This function is called once when autonomous is enabled. */
    @Override
    public void autonomousInit()
    {
        autoSelected = autoChooser.get();
        if (autoChooser != null)
        {
            System.out.println("Auto selected: " + autoSelected.getName());
            autoSelected.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic()
    {
        container.autonomousPeriodic();
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit()
    {
        if (autoSelected != null && autoSelected.isScheduled())
        {
            autoSelected.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic()
    {
        container.teleopPeroidic();
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit()
    {
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic()
    {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit()
    {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic()
    {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit()
    {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic()
    {
    }
}
