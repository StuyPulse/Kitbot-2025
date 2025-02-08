package com.stuypulse.robot.commands.dropper;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DropperFixSequence extends SequentialCommandGroup{

    public DropperFixSequence() {
        addCommands(
            new DropperDrop(),
            new WaitCommand(0.15),
            new DropperStop(),
            new DropperReverse(),
            new WaitCommand(0.3),
            new DropperStop()
        );
    }
}
