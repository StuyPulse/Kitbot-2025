package com.stuypulse.robot.commands;

import com.stuypulse.robot.commands.swerve.SwervePathFind;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Field.CoralBranch;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Test extends SequentialCommandGroup{
    
    public Test() {
        addCommands(
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.A)),
            SwervePathFind.toPose(Field.getTargetPoseForClosestCoralStation()),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.B)),
            SwervePathFind.toPose(Field.getTargetPoseForClosestCoralStation()),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.C)),
            SwervePathFind.toPose(Field.getTargetPoseForClosestCoralStation()),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.D)),
            SwervePathFind.toPose(Field.getTargetPoseForClosestCoralStation()),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.E)),
            SwervePathFind.toPose(Field.getTargetPoseForClosestCoralStation()),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.F)),
            SwervePathFind.toPose(Field.getTargetPoseForClosestCoralStation()),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.G)),
            SwervePathFind.toPose(Field.getTargetPoseForClosestCoralStation()),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.H)),
            SwervePathFind.toPose(Field.getTargetPoseForClosestCoralStation()),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.I)),
            SwervePathFind.toPose(Field.getTargetPoseForClosestCoralStation()),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.J)),
            SwervePathFind.toPose(Field.getTargetPoseForClosestCoralStation()),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.K)),
            SwervePathFind.toPose(Field.getTargetPoseForClosestCoralStation()),
            SwervePathFind.toPose(Field.getTargetPoseForCoralBranch(CoralBranch.L)),
            SwervePathFind.toPose(Field.getTargetPoseForClosestCoralStation())
        );
    }
}
