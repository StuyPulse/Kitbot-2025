package com.stuypulse.robot.commands.auton.tests;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StraightLineTest extends SequentialCommandGroup {
    
    public StraightLineTest(PathPlannerPath... paths) {

        addCommands(

            SwerveDrive.getInstance().followPathCommand(paths[0])
            
        );

    }

}
