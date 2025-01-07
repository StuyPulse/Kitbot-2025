package com.stuypulse.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwervePathFind{
    
    public static Command toPose(Pose2d pose) {
        Command command = AutoBuilder.pathfindToPose(pose, Settings.Swerve.DEFAULT_CONSTRAINTS)
            .until(() -> Odometry.getInstance().getPose().getTranslation().getDistance(pose.getTranslation()) < Settings.Swerve.XY_TOLERANCE);

        command.addRequirements(SwerveDrive.getInstance());
        return command;
    }
}
