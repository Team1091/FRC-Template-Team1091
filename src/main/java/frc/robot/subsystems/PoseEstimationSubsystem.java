package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import java.util.function.Supplier;

public class PoseEstimationSubsystem extends SubsystemBase {
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(.7,.7,9999999);
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Supplier<Rotation2d> rotationSupplier;
    private final Supplier<SwerveModulePosition[]> modulePositionSupplier;

    public PoseEstimationSubsystem(Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> modulePositionSupplier){
        this.rotationSupplier = rotationSupplier;
        this.modulePositionSupplier = modulePositionSupplier;

        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.kinematics,
                rotationSupplier.get(),
                modulePositionSupplier.get(),
                new Pose2d(),
                stateStdDevs,
                visionMeasurementStdDevs);
    }

    @Override
    public void periodic(){
        poseEstimator.update(rotationSupplier.get(), modulePositionSupplier.get());

        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        poseEstimator.addVisionMeasurement(
                limelightMeasurement.pose,
                limelightMeasurement.timestampSeconds);
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
    }

    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }
}