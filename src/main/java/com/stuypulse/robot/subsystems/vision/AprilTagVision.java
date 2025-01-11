package com.stuypulse.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AprilTagVision extends SubsystemBase {

    private static final AprilTagVision instance;

    static {
        instance = new LimelightVision();
    }

    public static AprilTagVision getInstance() {
        return instance;
    }

    public abstract void setTagWhitelist(int... ids);

    public abstract void setCameraEnabled(String name, boolean enabled);

    public abstract void disable();
    public abstract void enable();
}