package com.stuypulse.robot.subsystems.dropper;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

public class DropperImpl extends Dropper {
    
    private final SparkMax motor;

    public DropperImpl() {
        super();
        this.motor = new SparkMax(Ports.Dropper.MOTOR, MotorType.kBrushless);
        motor.configure(Motors.Dropper.MOTOR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setMotorBasedOnState() {
        switch (state) {
            case DROPPING:
                motor.set(Settings.Dropper.SCORE_SPEED);
                break;
            case STOP:
                motor.stopMotor();
                break;
        }
    }

    @Override
    public void periodic() {
        setMotorBasedOnState();
    }
    
    
}