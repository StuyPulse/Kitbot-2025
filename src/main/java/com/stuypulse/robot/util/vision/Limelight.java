package com.stuypulse.robot.util.vision;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.vision.LimelightHelpers.LimelightResults;
import com.stuypulse.robot.util.vision.LimelightHelpers.LimelightTarget_Fiducial;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.Optional;

public class Limelight extends SubsystemBase{

    private String name;

    private Optional<VisionData> data;
    
    private boolean enabled;

    public Limelight(String name, Pose3d robotRelativePose) {
        this.name = name;

        LimelightHelpers.setCameraPose_RobotSpace(
            name, 
            robotRelativePose.getX(), 
            robotRelativePose.getY(), 
            robotRelativePose.getZ(), 
            robotRelativePose.getRotation().getX(), 
            robotRelativePose.getRotation().getY(), 
            robotRelativePose.getRotation().getZ()
        );

        data = Optional.empty();

        enabled = true;
    }

    public Optional<VisionData> getData() {
        return data;
    }

    public boolean hasData() {
        return getData().isPresent();
    }

    public String getName() {
        return name;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    private int[] getIds(LimelightResults results) {
        return Arrays.stream(results.targets_Fiducials)
            .map((LimelightTarget_Fiducial fiducial) -> (int)fiducial.fiducialID)
            .mapToInt(Integer::intValue)
            .toArray();
    }

    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation(name, Odometry.getInstance().getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
        
        if (enabled) {
            LimelightResults latestResult = LimelightHelpers.getLatestResults(name);

            Pose3d pose = latestResult.getBotPose3d_wpiBlue();
            int[] ids = getIds(latestResult);
            double timestamp = latestResult.timestamp_LIMELIGHT_publish;
            
            for (int id : ids) {
                if (!Field.isValidAprilTagId(id)) {
                    data = Optional.empty();
                    return;
                }
            }

            data = Optional.of(new VisionData(pose, ids, timestamp));
        }
        else {
            data = Optional.empty();
        }
    }
}
