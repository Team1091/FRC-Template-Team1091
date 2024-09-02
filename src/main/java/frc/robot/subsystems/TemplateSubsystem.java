package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
    subsystems contain methods used to interact with the robot hardware
    each hardware component can only be accessed in one subsystem
    methods in subsystems are run by commands
 */

public class TemplateSubsystem extends SubsystemBase {

    //motors are defined by the type of their motor controller (ask electrical)
    private final PWMVictorSPX pwmMotor;
    private final CANSparkMax canMotor;
    private final Encoder encoder;
    private double speed;

    public TemplateSubsystem() {
        this.pwmMotor = new PWMVictorSPX(Constants.Template.pwmMotorChannel);
        this.canMotor = new CANSparkMax(Constants.Template.canMotorChannel, CANSparkLowLevel.MotorType.kBrushless);//must change brushless to brushed if using brushed motors (ask electrical)
        this.encoder = new Encoder(Constants.Template.encoderChannel1, Constants.Template.encoderChannel2);
    }

    public void resetEncoder() {
        encoder.reset();
    }

    public int getEncoderPosition() {
        return encoder.get();//value is in encoder counts (does not return common units like degrees)
    }

    public void setMotorSpeed(double speed) {
        this.speed = speed;
    }

    //the periodic method always runs over and over when robot is enabled
    @Override
    public void periodic() {
        //sets voltage from -1 to 1 not actual rpm
        pwmMotor.set(speed);
        canMotor.set(speed);
    }
}


