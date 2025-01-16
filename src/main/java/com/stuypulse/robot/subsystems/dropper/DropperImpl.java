package com.stuypulse.robot.subsystems.dropper;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

public class DropperImpl extends Dropper {
    
    private final SparkMax motor;

    public DropperImpl() {
        super();
        this.motor = new SparkMax(Ports.Dropper.MOTOR, MotorType.kBrushed);
        SparkBaseConfig motorConfig = new SparkMaxConfig().inverted(Settings.Dropper.inverted).idleMode(IdleMode.kBrake);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setMotorBasedOnState() {
        switch (state) {
            case DROPPING:
                motor.set(Settings.Dropper.DROP_SPEED);
                break;
            case REVERSING:
                motor.set(-Settings.Dropper.REVERSE_SPEED);
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