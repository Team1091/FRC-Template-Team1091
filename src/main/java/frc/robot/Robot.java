// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

//Runs everything
//Don't touch this if you don't have to
public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    //Actions run when robot is powered on
    @Override
    public void robotInit() {
        //starts robot
        m_robotContainer = new RobotContainer();

        //starts camera with settings
        CameraServer.startAutomaticCapture().setExposureManual(40);

        //warms up path planner to avoid delay on first path
        FollowPathCommand.warmupCommand().schedule();
    }

    //Runs actions over and over when robot is powered on
    @Override
    public void robotPeriodic() {
        //Runs every periodic function besides this one
        CommandScheduler.getInstance().run();
    }

    //Runs actions when robot is disabled
    @Override
    public void disabledInit() {

    }

    //Runs actions over and over while robot is disabled
    @Override
    public void disabledPeriodic() {

    }

    //Runs actions when robot is enabled
    @Override
    public void disabledExit() {
        m_robotContainer.robotEnabled();
    }


    //Runs autos
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    //Runs over and over while in auto
    @Override
    public void autonomousPeriodic() {

    }

    //Runs actions when switching from auto to teleop
    @Override
    public void teleopInit() {
        //Makes sure auto stops when teleop starts
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    //Runs actions over and over while in teleop
    @Override
    public void teleopPeriodic() {

    }

    //Runs when put into test mode
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    //Runs actions over and over while in test mode
    @Override
    public void testPeriodic() {

    }

    //Runs when put into simulation mode
    public void simulationInit() {
    }

    //Runs actions over and over while in simulation mode
    @Override
    public void simulationPeriodic() {
    }
}