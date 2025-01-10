package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.util.vision.Limelight;
import com.stuypulse.robot.util.vision.VisionData;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class LimelightVision extends AprilTagVision{

    private ArrayList<VisionData> outputs;
    private Limelight[] limelights;

    private boolean enabled;

    public LimelightVision() {
        limelights = new Limelight[Cameras.LimelightCameras.length];
        for (int i = 0; i < limelights.length; i++) {
            limelights[i] = new Limelight(Cameras.LimelightCameras[i].getName(), Cameras.LimelightCameras[i].getLocation());
        }

        outputs = new ArrayList<VisionData>();

        enabled = true;
    }

    @Override
    public ArrayList<VisionData> getOutputs() {
        return outputs;
    }

    @Override
    public void setTagWhitelist(int... ids) {
        for (Limelight limelight : limelights) {
            limelight.setTagWhitelist(ids);
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
        for (Limelight limelight : limelights) {
            if (limelight.getName().equals(name)) {
                limelight.setEnabled(enabled);
            }
        }
    }

    @Override
    public void periodic() {
        outputs.clear();

        if (enabled) {
            for (Limelight limelight : limelights) {
                if (limelight.hasData()) {
                    outputs.add(limelight.getData().get());
                }
            }
        }

        SmartDashboard.putBoolean("Vision/Has Any Data", outputs.size() > 0);
    }
}
