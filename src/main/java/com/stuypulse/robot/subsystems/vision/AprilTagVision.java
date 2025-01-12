package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AprilTagVision extends SubsystemBase {

    private static final AprilTagVision instance;

    static {
        if (Robot.isReal()) {
            instance = new LimelightVision();
        }
        else {
            instance = new SimVision();
        }
    }

    public static AprilTagVision getInstance() {
        return instance;
    }

    public abstract void setTagWhitelist(int... ids);

    public abstract void setCameraEnabled(String name, boolean enabled);

    public abstract void disable();
    public abstract void enable();
}