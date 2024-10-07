package frc.robot.commands.autoCommands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.TemplateSubsystem;
import frc.robot.subsystems.drive.Drive;


public class Auto2Command {

    public static Command create(Drive drive, TemplateSubsystem templateSubsystem) {
        return new SequentialCommandGroup(

        );
    }

}
