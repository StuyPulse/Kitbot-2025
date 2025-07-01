package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SeedFieldRelative extends InstantCommand{
    
    private final SwerveDrive swerve;

    public SeedFieldRelative() {
        this.swerve = SwerveDrive.getInstance();
    }

    @Override
    public void initialize() {
        swerve.resetHeading();
    }
}
