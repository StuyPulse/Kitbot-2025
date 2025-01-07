package com.stuypulse.robot.subsystems.dropper;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartNumber;

public class DropperSim extends Dropper {
    
    private final SmartNumber motor;

    public DropperSim() {
        super();
        this.motor = new SmartNumber("Dropper/Speed", 0);
    }

    private void setMotorBasedOnState() {
        switch (state) {
            case DROPPING:
                motor.set(Settings.Dropper.DROP_SPEED);
                break;
            case STOP:
                motor.set(0);
                break;
        }
    }

    @Override
    public void periodic() {
        setMotorBasedOnState();
    }
    
    
}