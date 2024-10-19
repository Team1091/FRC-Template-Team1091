// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.commands.autoCommands.Auto1Command;
import frc.robot.commands.autoCommands.Auto2Command;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.TemplateSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.Constants.Swerve.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import org.photonvision.PhotonCamera;


public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final PoseEstimationSubsystem poseEstimationSubsystem;
    private final TemplateSubsystem templateSubsystem = new TemplateSubsystem();

    //Miscellaneous
    private final SendableChooser<AutoChoice> autoChooser = new SendableChooser<>();

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController secondDriver = new CommandXboxController(1);


    //Define stuff in here
    public RobotContainer() {
        drive = new Drive(
                new GyroIOPigeon2(),//change if using different gyro
                new ModuleIOTalonFX(FRONT_LEFT),
                new ModuleIOTalonFX(FRONT_RIGHT),
                new ModuleIOTalonFX(BACK_LEFT),
                new ModuleIOTalonFX(BACK_RIGHT)
        );
        poseEstimationSubsystem = new PoseEstimationSubsystem(
                drive::getRotation,
                drive::getModulePositions
        );

        //Add in named commands for pathplanner
        NamedCommands.registerCommand("Template", new TemplateCommand(templateSubsystem, Constants.Template.motorSpeed));

        configureButtonBindings();

        //Add autos to shuffleboard
        autoChooser.addOption("Auto 1", AutoChoice.Auto1);
        autoChooser.addOption("Auto 2", AutoChoice.Auto2);
        Shuffleboard.getTab("General").add("Auto Choice", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    //Actions run when robot is enabled
    public void robotEnabled() {
        poseEstimationSubsystem.setCurrentPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
        drive.straightenWheels();
        drive.resetGyro();
        drive.setFieldState(true);

        templateSubsystem.resetEncoder();
    }


    //Map commands to controller buttons
    private void configureButtonBindings() {

        //Template
        driver.a().whileTrue(new TemplateCommand(templateSubsystem, Constants.Template.motorSpeed));

        //Drive
        driver.povUp().onTrue(Commands.runOnce(() -> poseEstimationSubsystem.setCurrentPose(new Pose2d(poseEstimationSubsystem.getCurrentPose().getTranslation(), new Rotation2d())), poseEstimationSubsystem));
        driver.povLeft().onTrue(runOnce(drive::toggleIsFieldOriented));

        drive.setDefaultCommand(
                DriveCommand.joystickDrive(
                        drive,
                        () -> { // x+ forward is front, x- is backward
//                            return joystick.getY();
                            return driver.getLeftY();
                        },
                        () -> { // y+ is to the left, y- is to the right
//                            return -joystick.getX();
                            return -driver.getLeftX();
                        },
                        () -> { // z+ is rotating counterclockwise
//                            return -joystick.getTwist();
                            return -driver.getRightX();
                        }
                )
        );
    }


    //Pass pathplanner autos to robot
    public Command getAutonomousCommand() {
        AutoChoice autoChoice = autoChooser.getSelected();
        Command command;

        switch (autoChoice) {
            case Auto1:
                command = Auto1Command.create(drive, templateSubsystem);
                break;
            case Auto2:
                command = Auto2Command.create(drive, templateSubsystem);
                break;
            default:
                command = Auto1Command.create(drive, templateSubsystem);
        }

        return new ParallelCommandGroup(command);
    }
}


