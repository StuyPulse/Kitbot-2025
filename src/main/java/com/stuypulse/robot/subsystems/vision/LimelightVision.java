package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.vision.LimelightHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;

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

    @Override
    public void periodic() {
        if (enabled) {
            for (int i = 0; i < names.length; i++) {
                if (camerasEnabled[i]) {
                    String limelightName = names[i];
                    Pose2d robotPose = LimelightHelpers.getBotPose2d(limelightName);
                    double timestamp = LimelightHelpers.getLatestResults(limelightName).timestamp_LIMELIGHT_publish;

                    if (robotPose.getTranslation().getDistance(new Translation2d()) != 0) {
                        Odometry.getInstance().addVisionData(robotPose, timestamp);
                    }
                }
            }
        }
    }
}
 