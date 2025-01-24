package com.stuypulse.robot.commands.dropper;

import com.stuypulse.robot.subsystems.dropper.Dropper;
import com.stuypulse.robot.subsystems.dropper.Dropper.State;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DropperDrop extends InstantCommand{

    private Dropper dropper;

    public DropperDrop() {
        this.dropper = Dropper.getInstance();
        addRequirements(dropper);
    }

    @Override
    public void initialize() {
        dropper.setState(State.DROPPING);
    }
}
