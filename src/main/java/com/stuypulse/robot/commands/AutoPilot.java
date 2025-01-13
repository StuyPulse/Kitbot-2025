package com.stuypulse.robot.commands;

import java.util.ArrayList;

import com.stuypulse.robot.commands.dropper.DropperShootSequence;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveAwayFromReef;
import com.stuypulse.robot.commands.swerve.SwervePathFind;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Field.CoralBranch;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoPilot extends SequentialCommandGroup{
    
    public AutoPilot() {
        ArrayList<Command> commands = new ArrayList<>();
        for (CoralBranch branch : CoralBranch.values()) {
            Command command = new SequentialCommandGroup(
                SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(branch).transformBy(new Transform2d(-1.0, 0, new Rotation2d())))
                    .onlyIf(() -> !branch.onDriverStationSide()),
                SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(branch)),
                new DropperShootSequence(),
                SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(branch).transformBy(new Transform2d(-1.0, 0, new Rotation2d())))
                    .onlyIf(() -> !branch.onDriverStationSide()),
                SwervePathFind.toNearestCoralStation(),
                new WaitCommand(Settings.Dropper.WAIT_TIME_AT_CORAL_STATION)
            );
            commands.add(command);
        }

        addCommands(commands.toArray(new Command[0]));
    }
}
