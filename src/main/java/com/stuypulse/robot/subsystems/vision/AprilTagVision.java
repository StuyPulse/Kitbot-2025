package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.vision.VisionData;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

public abstract class AprilTagVision extends SubsystemBase {

    private static final AprilTagVision instance;

    static {
        switch (Settings.Vision.VISION_TYPE) {
            case LIMELIGHT:
                instance = new LimelightVision();
                break;
            case PHOTON_VISION:
                instance = new PhotonVision();
                break;
            default:
                instance = new PhotonVision();
                break;
        }
    }

    public static AprilTagVision getInstance() {
        return instance;
    }

    public abstract ArrayList<VisionData> getOutputs();

    public abstract void setTagWhitelist(int... ids);

    public abstract void setCameraEnabled(String name, boolean enabled);

    public abstract void disable();
    public abstract void enable();
}