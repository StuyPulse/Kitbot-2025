package com.stuypulse.robot.commands.dropper;

import com.stuypulse.robot.subsystems.dropper.Dropper;
import com.stuypulse.robot.subsystems.dropper.Dropper.State;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DropperReverse extends InstantCommand{

    private Dropper dropper;

    public DropperReverse() {
        this.dropper = Dropper.getInstance();
        addRequirements(dropper);
    }

    @Override
    public void initialize() {
        dropper.setState(State.REVERSING);
    }
}
