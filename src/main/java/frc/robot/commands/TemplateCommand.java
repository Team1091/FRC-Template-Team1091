package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TemplateSubsystem;

/*
    commands handle inputs and turn them into usable values for subsystems
    multiple commands can all use the same subsystem
    they only run when called usually by a button
 */

public class TemplateCommand extends Command {
    private final TemplateSubsystem templateSubsystem;
    private final double motorSpeed;

    public TemplateCommand(TemplateSubsystem templateSubsystem, double motorSpeed) {
        this.templateSubsystem = templateSubsystem;
        this.motorSpeed = motorSpeed;
        addRequirements(templateSubsystem);
    }

    //runs once when command is called
    @Override
    public void initialize() {
    }

    //runs while command is being called
    @Override
    public void execute() {
        templateSubsystem.setMotorSpeed(motorSpeed);
    }

    //runs once when command is stopped
    @Override
    public void end(boolean interrupted) {
        templateSubsystem.setMotorSpeed(0);
    }

    //returns conditions for when command should be stopped (command stops when it returns true)
    @Override
    public boolean isFinished() {
        return false;
    }
}

