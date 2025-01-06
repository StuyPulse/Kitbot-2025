package com.stuypulse.robot.subsystems.odometry;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.util.vision.VisionData;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;


public class OdometryImpl extends Odometry {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveOdometry odometry;
    private final Field2d field;

    private final FieldObject2d odometryPose2d;
    private final FieldObject2d poseEstimatorPose2d;

    protected OdometryImpl() {
        var swerve = SwerveDrive.getInstance();
        var startingPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        poseEstimator =
            new SwerveDrivePoseEstimator(
                swerve.getKinematics(),
                swerve.getGyroAngle(),
                swerve.getModulePositions(),
                startingPose,

                VecBuilder.fill(
                    0.1,
                    0.1,
                    0.1),

                Settings.Vision.STDDEVS);

        odometry =
            new SwerveDriveOdometry(
                swerve.getKinematics(),
                swerve.getGyroAngle(),
                swerve.getModulePositions(),
                startingPose);

        field = new Field2d();

        odometryPose2d = field.getObject("Odometry Pose2d");
        poseEstimatorPose2d = field.getObject("Pose Estimator Pose2d");

        swerve.initFieldObjects(field);
        SmartDashboard.putData("Field", field);
    }

    @Override
    public Pose2d getPose() {
        if (USE_VISION_ANGLE.get())
            return poseEstimator.getEstimatedPosition();
        else
            return new Pose2d(
                poseEstimator.getEstimatedPosition().getTranslation(),
                odometry.getPoseMeters().getRotation());
    }

    @Override
    public void reset(Pose2d pose) {
        SwerveDrive drive = SwerveDrive.getInstance();

        poseEstimator.resetPosition(
            drive.getGyroAngle(),
            drive.getModulePositions(),
            pose);

        odometry.resetPosition(
            drive.getGyroAngle(),
            drive.getModulePositions(),
            pose);
    }

    @Override
    public Field2d getField() {
        return field;
    }

    private void processResults(List<VisionData> results){
        for (VisionData result : results) {
            poseEstimator.addVisionMeasurement(result.getPose().toPose2d(), result.getTimestamp());
        }
    }

    @Override
    public void periodic() {
        SwerveDrive drive = SwerveDrive.getInstance();
        poseEstimator.update(drive.getGyroAngle(), drive.getModulePositions());
        odometry.update(drive.getGyroAngle(), drive.getModulePositions());

        poseEstimatorPose2d.setPose(poseEstimator.getEstimatedPosition());

        AprilTagVision vision = AprilTagVision.getInstance();
        List<VisionData> results = vision.getOutputs();
        processResults(results);

        odometryPose2d.setPose(odometry.getPoseMeters());

        SmartDashboard.putNumber("Odometry/Odometry Pose X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry/Odometry Pose Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odometry/Odometry Rotation", odometry.getPoseMeters().getRotation().getDegrees());

        SmartDashboard.putNumber("Odometry/Pose Estimator Pose X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Odometry/Pose Estimator Pose Y", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Odometry/Pose Estimator Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    }
}