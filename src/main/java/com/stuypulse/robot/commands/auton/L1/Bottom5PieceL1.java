package com.stuypulse.robot.commands.auton.L1;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.dropper.DropperDrop;
import com.stuypulse.robot.commands.dropper.DropperShootSequence;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Bottom5PieceL1 extends SequentialCommandGroup {
    
    public Bottom5PieceL1(PathPlannerPath... paths) {

        addCommands(

            // Start in front of Side 3, Drive to + Score on Side 3, Preload
            SwerveDrive.getInstance().followPathCommand(paths[0]),
            new DropperShootSequence(),

            // To HP to Side 4, Piece 2
            SwerveDrive.getInstance().followPathCommand(paths[1]),
            SwerveDrive.getInstance().followPathCommand(paths[2]),
            new DropperShootSequence(),

            // To HP to Side 4, Piece 3
            SwerveDrive.getInstance().followPathCommand(paths[3]),
            SwerveDrive.getInstance().followPathCommand(paths[4]),
            new DropperShootSequence(),

            // To HP to Side 4, Piece 4
            SwerveDrive.getInstance().followPathCommand(paths[5]),
            SwerveDrive.getInstance().followPathCommand(paths[6]),
            new DropperShootSequence(),

            // To HP to Side 4, Piece 5
            SwerveDrive.getInstance().followPathCommand(paths[7]),
            SwerveDrive.getInstance().followPathCommand(paths[8]),
            new DropperShootSequence()

        ); 

    }

}