package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TemplateSubsystem;

public class TemplateCommand extends Command {
    private final TemplateSubsystem templateSubsystem;
    private final double speed;

    public TemplateCommand(TemplateSubsystem templateSubsystem, double speed) {
        this.templateSubsystem = templateSubsystem;
        this.speed = speed;
        addRequirements(templateSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        templateSubsystem.setPickUpMotorSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        templateSubsystem.setPickUpMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

