package com.stuypulse.robot.commands.swerve;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class SwervePathFind{
    
    public static Command toPose(Pose2d pose) {
        Command command = AutoBuilder.pathfindToPose(pose, Settings.Swerve.DEFAULT_CONSTRAINTS);
        command.addRequirements(SwerveDrive.getInstance());
        return command;
    }

    public static Command toNearestCoralStation() {
        return new ConditionalCommand(
            toPose(Field.getTargetPoseForCDCoralStation()), 
            toPose(Field.getTargetPoseForKLCoralStation()), 
            () -> Field.robotIsCloserToCDCoralStation()
        );
    }
}
