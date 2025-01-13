package com.stuypulse.robot.subsystems.swerve.modules;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Controllers.Modules;
import com.stuypulse.robot.constants.Settings.Swerve.ModuleOffsets.BackLeft;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive.ModulePosition;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SacrodModule extends SwerveModule {

    // module data

    private final String name;
    private final Translation2d location;

    private SwerveModuleState targetState;

    // turn

    private final SparkMax turnMotor;
    private final CANcoder absoluteEncoder;
    private final Angle angleOffset;

    private final AngleController turnController;

    // drive
    private final SparkMax driveMotor;
    private final RelativeEncoder driveEncoder;

    private final Controller driveController;

    public SacrodModule(ModulePosition position) {
        this.name = position.toString();
        this.location = Settings.Swerve.ModuleOffsets.getXYOffset(position);
        this.angleOffset = Settings.Swerve.ModuleOffsets.getAngleOffset(position);

        this.driveController = new PIDController(Modules.Drive.kP, Modules.Drive.kI, Modules.Drive.kD)
                                .add(new MotorFeedforward(Modules.Drive.kS, Modules.Drive.kV, Modules.Drive.kA).velocity());
        this.turnController = new AnglePIDController(Modules.Turn.kP, Modules.Turn.kI, Modules.Turn.kD);

        targetState = new SwerveModuleState();

        switch (position) {
            case FRONT_LEFT:
                turnMotor = new SparkMax(Ports.Swerve.Turn.FRONT_LEFT, MotorType.kBrushless);
                turnMotor.configure(Motors.Swerve.Turn.FRONT_LEFT, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                driveMotor = new SparkMax(Ports.Swerve.Drive.FRONT_LEFT, MotorType.kBrushless);
                driveMotor.configure(Motors.Swerve.Drive.FRONT_LEFT, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                driveEncoder = driveMotor.getEncoder();
                absoluteEncoder = new CANcoder(Ports.Swerve.CANCoderIds.FRONT_LEFT);
                break;
            case FRONT_RIGHT:
                turnMotor = new SparkMax(Ports.Swerve.Turn.FRONT_RIGHT, MotorType.kBrushless);
                turnMotor.configure(Motors.Swerve.Turn.FRONT_RIGHT, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                driveMotor = new SparkMax(Ports.Swerve.Drive.FRONT_RIGHT, MotorType.kBrushless);
                driveMotor.configure(Motors.Swerve.Drive.FRONT_RIGHT, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                driveEncoder = driveMotor.getEncoder();
                absoluteEncoder = new CANcoder(Ports.Swerve.CANCoderIds.FRONT_RIGHT);
                break;
            case BACK_LEFT:
                turnMotor = new SparkMax(Ports.Swerve.Turn.BACK_LEFT, MotorType.kBrushless);
                turnMotor.configure(Motors.Swerve.Turn.BACK_LEFT, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                driveMotor = new SparkMax(Ports.Swerve.Drive.BACK_LEFT, MotorType.kBrushless);
                driveMotor.configure(Motors.Swerve.Drive.BACK_LEFT, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                driveEncoder = driveMotor.getEncoder();
                absoluteEncoder = new CANcoder(Ports.Swerve.CANCoderIds.BACK_LEFT);
                break;
            case BACK_RIGHT:
                turnMotor = new SparkMax(Ports.Swerve.Turn.BACK_RIGHT, MotorType.kBrushless);
                turnMotor.configure(Motors.Swerve.Turn.BACK_RIGHT, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                driveMotor = new SparkMax(Ports.Swerve.Drive.BACK_RIGHT, MotorType.kBrushless);
                driveMotor.configure(Motors.Swerve.Drive.BACK_RIGHT, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                driveEncoder = driveMotor.getEncoder();
                absoluteEncoder = new CANcoder(Ports.Swerve.CANCoderIds.BACK_RIGHT);
                break;
            default:
                throw new IllegalArgumentException();
        }
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public Translation2d getOffset() {
        return location;
    }

    @Override
    public void setTargetState(SwerveModuleState state) {
        state.optimize(getRotation2d());
        targetState = state;
    }

    private double getSpeed() {
        return driveEncoder.getVelocity();
    }

    private double getDistance() {
        return driveEncoder.getPosition();
    }

    private Rotation2d getAbsolutePosition() {
        return new Rotation2d(MathUtil.interpolate(-Math.PI, +Math.PI, (absoluteEncoder.getAbsolutePosition().getValueAsDouble() + 1) / 2));
    }

    private Rotation2d getRotation2d() {
        return getAbsolutePosition().minus(angleOffset.getRotation2d());
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), getRotation2d());
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistance(), getRotation2d());
    }

    @Override
    public void periodic() {
        turnMotor.setVoltage(turnController.update(
                Angle.fromRotation2d(targetState.angle),
                Angle.fromRotation2d(getRotation2d())));
        driveMotor.setVoltage(driveController.update(targetState.speedMetersPerSecond, getSpeed()));

        SmartDashboard.putNumber("Swerve/" + name + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/" + name + "/Angle", getRotation2d().getDegrees());
        SmartDashboard.putNumber("Swerve/" + name + "/Angle Error", turnController.getError().toDegrees());
        SmartDashboard.putNumber("Swerve/" + name + "/Angle Voltage", turnController.getOutput());
        SmartDashboard.putNumber("Swerve/" + name + "/Absolute Angle", getAbsolutePosition().getDegrees());
        SmartDashboard.putNumber("Swerve/" + name + "/Angle Current", turnMotor.getOutputCurrent());

        SmartDashboard.putNumber("Swerve/" + name + "/Target Speed", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/" + name + "/Velocity", getSpeed());
        SmartDashboard.putNumber("Swerve/" + name + "/Velocity Error", driveController.getError());
        SmartDashboard.putNumber("Swerve/" + name + "/Velocity Voltage", driveController.getOutput());
        SmartDashboard.putNumber("Swerve/" + name + "/Velocity Current", driveMotor.getOutputCurrent());
    }
}