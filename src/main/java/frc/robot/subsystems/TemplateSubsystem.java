package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class TemplateSubsystem extends SubsystemBase {

    private final PWMVictorSPX motor;
    private double speed;

    public TemplateSubsystem() {
        this.motor = new PWMVictorSPX(Constants.Template.motorChannel);
    }

    public void setPickUpMotorSpeed(double speed) {
        this.speed = speed;
    }

    @Override
    public void periodic() {
        motor.set(speed);
    }
}


