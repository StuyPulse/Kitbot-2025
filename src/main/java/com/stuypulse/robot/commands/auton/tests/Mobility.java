package com.stuypulse.robot.commands.auton.tests;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Mobility extends SequentialCommandGroup {

    public Mobility(PathPlannerPath... path) {
        
        addCommands(

            // Starts in front of Side 2, drives froward
            SwerveDrive.getInstance().followPathCommand(path[0])

        );

    }

}