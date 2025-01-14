package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SeedFieldRelative extends InstantCommand{
    
    private final Odometry odometry;

    public SeedFieldRelative() {
        this.odometry = Odometry.getInstance();
    }

    @Override
    public void initialize() {
        odometry.seedFieldRelative();
    }
}
