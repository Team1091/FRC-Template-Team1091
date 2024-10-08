package frc.robot;

import static frc.robot.Constants.Swerve.driveBaseRadius;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import java.util.Map;

public final class Constants {

    public static class Swerve {
        public static final int FRONT_LEFT = 0;
        public static final int FRONT_RIGHT = 1;
        public static final int BACK_LEFT = 2;
        public static final int BACK_RIGHT = 3;
        public static final double trackWidthX = Units.inchesToMeters(25.0);
        public static final double trackWidthY = Units.inchesToMeters(25.0);
        public static final double driveBaseRadius = Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
        public static final double maxLinearSpeed = Units.feetToMeters(14.5);
        public static final double maxLinearAcceleration = Units.feetToMeters(10);
        public static final double maxAngularSpeed = maxLinearSpeed / driveBaseRadius;
        public static final double maxAngularAcceleration = maxLinearAcceleration / driveBaseRadius;
    }

    public static final class Template {
        public final static int pwmMotorChannel = 1;
        public final static int canMotorChannel = 2;
        public final static int encoderChannel1 = 1;
        public final static int encoderChannel2 = 2;
        public final static double motorSpeed = 1;
    }

    public static final class PathPlanner {
        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(.5, 0, 0), // 2.0 Translation constants 3
                new PIDConstants(3, 0, 0), // 1.3 Rotation constants 3
                Swerve.maxLinearSpeed,
                driveBaseRadius, // Drive base radius (distance from center to furthest module)
                new ReplanningConfig());

    }
    
    public static final double linearDeadband = 0.1;
    public static final double rotationalDeadband = .1;

}
