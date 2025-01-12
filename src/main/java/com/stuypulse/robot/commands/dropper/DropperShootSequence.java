package com.stuypulse.robot.commands.dropper;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DropperShootSequence extends SequentialCommandGroup{

    public DropperShootSequence() {
        addCommands(
            new DropperDrop(),
            new WaitCommand(Settings.Dropper.DROP_TIME),
            new DropperStop()
        );
    }
}
