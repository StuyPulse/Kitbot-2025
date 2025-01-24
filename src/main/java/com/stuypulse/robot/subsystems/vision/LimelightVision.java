package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.vision.LimelightHelpers;
import com.stuypulse.robot.util.vision.LimelightHelpers.PoseEstimate;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightVision extends AprilTagVision{

    private String[] names;
    private boolean enabled;
    private boolean[] camerasEnabled;

    public LimelightVision() {
        names = new String[Cameras.LimelightCameras.length];
        for (int i = 0; i < Cameras.LimelightCameras.length; i++) {
            names[i] = Cameras.LimelightCameras[i].getName();
            Pose3d robotRelativePose = Cameras.LimelightCameras[i].getLocation();
            LimelightHelpers.setCameraPose_RobotSpace(
                names[i], 
                robotRelativePose.getX(), 
                robotRelativePose.getY(), 
                robotRelativePose.getZ(), 
                robotRelativePose.getRotation().getX(), 
                robotRelativePose.getRotation().getY(), 
                robotRelativePose.getRotation().getZ()
            );
        }

        camerasEnabled = new boolean[Cameras.LimelightCameras.length];
        for (int i = 0; i < camerasEnabled.length; i++) {
            camerasEnabled[i] = true;
        }

        enabled = true;
    }

    @Override
    public void setTagWhitelist(int... ids) {
        for (String name : names) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, ids);
        }
    }

    @Override
    public void enable() {
        enabled = true;
    }

    @Override
    public void disable() {
        enabled = false;
    }

    @Override
    public void setCameraEnabled(String name, boolean enabled) {
        for (int i = 0; i < names.length; i++) {
            if (names[i].equals(name)) {
                camerasEnabled[i] = enabled;
            }
        }
    }

    private Vector<N3> calculateSTDEVS(double avgTagDistance) {
        return Settings.Vision.MIN_STDEV.plus(
            VecBuilder.fill(1.0, 1.0, 2).times(avgTagDistance)
        );
    }

    @Override
    public void periodic() {
        if (enabled) {
            for (int i = 0; i < names.length; i++) {
                if (camerasEnabled[i]) {
                    String limelightName = names[i];
                    PoseEstimate poseEstimate = Robot.isBlue() 
                        ? LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName)
                        : LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);
                    
                    if (poseEstimate != null && poseEstimate.tagCount > 0 && poseEstimate.avgTagDist < Settings.Vision.DISTANCE_CUTOFF.get()) {
                        Pose2d robotPose = poseEstimate.pose;
                        double timestamp = poseEstimate.timestampSeconds;

                        Odometry.getInstance().addVisionData(robotPose, timestamp, calculateSTDEVS(poseEstimate.avgTagDist));
                    }
                }
            }
        }
    }
}
 