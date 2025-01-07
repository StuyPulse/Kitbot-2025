package com.stuypulse.robot.commands;

import com.stuypulse.robot.commands.swerve.SwervePathFind;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Field.CoralBranch;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Test extends SequentialCommandGroup{
    
    public Test() {
        addCommands(
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.A)),
            SwervePathFind.toNearestCoralStation(),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.B)),
            SwervePathFind.toNearestCoralStation(),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.C)),
            SwervePathFind.toNearestCoralStation(),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.D)),
            SwervePathFind.toNearestCoralStation(),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.E)),
            SwervePathFind.toNearestCoralStation(),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.F)),
            SwervePathFind.toNearestCoralStation(),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.G)),
            SwervePathFind.toNearestCoralStation(),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.H)),
            SwervePathFind.toNearestCoralStation(),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.I)),
            SwervePathFind.toNearestCoralStation(),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.J)),
            SwervePathFind.toNearestCoralStation(),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.K)),
            SwervePathFind.toNearestCoralStation(),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.L)),
            SwervePathFind.toNearestCoralStation()
        );
    }
}
