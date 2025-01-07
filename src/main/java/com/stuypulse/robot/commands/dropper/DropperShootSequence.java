package com.stuypulse.robot.commands.dropper;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.dropper.Dropper;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DropperShootSequence extends SequentialCommandGroup{
    private final Dropper dropper;

    public DropperShootSequence() {
        dropper = Dropper.getInstance();
        addCommands(
            new DropperDrop(),
            new WaitCommand(Settings.Dropper.DROP_TIME),
            new DropperStop()
        );
    }
}
