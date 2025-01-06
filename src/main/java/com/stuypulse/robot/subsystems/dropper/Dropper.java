package com.stuypulse.robot.subsystems.dropper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Dropper extends SubsystemBase {
    public static final Dropper instance;

    static {
        instance = new DropperImpl();
    }
    
    public static Dropper getInstance() {
        return instance;
    }

    public enum State {
        DROPPING,
        STOP
    }

    protected State state;

    protected Dropper() {
        this.state = State.STOP;
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return this.state;
    }
}