package frc.robot.subsystems.drive;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.PoseEstimationSubsystem;

import java.util.Optional;

import static frc.robot.Constants.Swerve.*;


// import frc.robot.util.LocalADStarAK;


public class Drive extends SubsystemBase {

    private static final double MAX_LINEAR_SPEED = Constants.Swerve.maxLinearSpeed;
    private static final double TRACK_WIDTH_X = Constants.Swerve.trackWidthX;
    private static final double TRACK_WIDTH_Y = Constants.Swerve.trackWidthY;
    private static final double DRIVE_BASE_RADIUS = Constants.Swerve.driveBaseRadius;
    private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    private final PoseEstimationSubsystem poseEstimationSubsystem;
    private final GyroIO gyroIO;
    private final GyroIO.GyroIOInputs gyroInputs = new GyroIO.GyroIOInputs();
    private GenericEntry gyroYawDebug;
    private GenericEntry gyroRollDebug;
    private GenericEntry gyroPitchDebug;
    private GenericEntry poseXDebug;
    private GenericEntry poseYDebug;
    private GenericEntry poseRotDebug;
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR


    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Pose2d pose = new Pose2d();
    private Rotation2d lastGyroRotation = new Rotation2d();
    private StructArrayPublisher<SwerveModuleState> publisher;
    private boolean isFieldOriented = true;
    private ChassisSpeeds robotRelativeSpeeds;

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO, PoseEstimationSubsystem poseEstimationSubsystem) {
        this.gyroIO = gyroIO;
        this.poseEstimationSubsystem = poseEstimationSubsystem;
        AutoBuilder.configureHolonomic(
            poseEstimationSubsystem::getCurrentPose, // Robot pose supplier
            poseEstimationSubsystem::setCurrentPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::runVelocity, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            Constants.pathFollowerConfig,
                    () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    },
            this // Reference to this subsystem to set requirements
            );

        modules[FRONT_LEFT] = new Module(flModuleIO, 0, "FL");
        modules[FRONT_RIGHT] = new Module(frModuleIO, 1, "FR");
        modules[BACK_LEFT] = new Module(blModuleIO, 2, "BL");
        modules[BACK_RIGHT] = new Module(brModuleIO, 3, "BR");

        publisher = NetworkTableInstance
                .getDefault()
                .getStructArrayTopic("MyStates", SwerveModuleState.struct)
                .publish();
        gyroYawDebug = Shuffleboard.getTab("General").add("Gyro Yaw", Optional.of(0)).getEntry();
        gyroRollDebug = Shuffleboard.getTab("General").add("Gyro Roll", Optional.of(0)).getEntry();
        gyroPitchDebug = Shuffleboard.getTab("General").add("Gyro Pitch", Optional.of(0)).getEntry();
        poseXDebug = Shuffleboard.getTab("General").add("Pose X", Optional.of(0)).getEntry();
        poseYDebug = Shuffleboard.getTab("General").add("Pose Y", Optional.of(0)).getEntry();
        poseRotDebug = Shuffleboard.getTab("General").add("Pose Rotation", Optional.of(0)).getEntry();
        

    }

    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        for (var module : modules) {
            module.periodic();
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }
    }

    /**
     * Runs the drive at the desired velocity.
     *
//     * @param speeds Speeds in meters/sec
     */

        public void setOrientation(Translation2d linearVelocity, double omega){
            Rotation2d rotation;
            if (isFieldOriented) {
                rotation = poseEstimationSubsystem.getCurrentPose().getRotation();
            } else {
                rotation = new Rotation2d(0);
            }
             robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearVelocity.getX() * MAX_LINEAR_SPEED,
                    linearVelocity.getY() * MAX_LINEAR_SPEED,
                    omega * MAX_ANGULAR_SPEED,
                    rotation
            );
            runVelocity(robotRelativeSpeeds);
        }

        public void runVelocity(ChassisSpeeds chassisSpeeds) {

        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
        }

        optimizedSetpointStates = setpointStates;

        publisher.set(optimizedSetpointStates);
    }

     public Command followPathCommand(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return new FollowPathHolonomic(
            path,
            poseEstimationSubsystem::getCurrentPose, // Robot pose supplier
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::runVelocity, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

    public ChassisSpeeds getRobotRelativeSpeeds()
    {
        return robotRelativeSpeeds;
    }

    /**
     * Stops the drive.
     */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public void setFieldState(boolean bool){
        isFieldOriented = bool;
    }
    public void toggleIsFieldOriented() {
        isFieldOriented = !isFieldOriented;
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    public void straightenWheels() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = new Rotation2d(0);
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /**
     * Runs forwards at the commanded voltage.
     */
    public void runCharacterizationVolts(double volts) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(volts);
        }
    }

    /**
     * Returns the average drive velocity in radians/sec.
     */
    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (var module : modules) {
            driveVelocityAverage += module.getCharacterizationVelocity();
        }
        return driveVelocityAverage / 4.0;
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the modules.
     */
//    @AutoLogOutput(key = "SwerveStates/Measured")
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    public void resetGyro() {
        gyroIO.resetGyro();
    }

    public Rotation2d getGyroRotation() {return new Rotation2d(gyroInputs.yawPosition.getRadians());}

    /**
     * Returns the maximum linear speed in meters per sec.
     */
    public double getMaxLinearSpeedMetersPerSec() {
        return MAX_LINEAR_SPEED;
    }

    /**
     * Returns the maximum angular speed in radians per sec.
     */
    public double getMaxAngularSpeedRadPerSec() {
        return MAX_ANGULAR_SPEED;
    }

    /**
     * Returns an array of module translations.
     */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[]{
                new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0), // FL
                new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0), // FR
                new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0), // BL
                new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0) // BR
        };
    }
}