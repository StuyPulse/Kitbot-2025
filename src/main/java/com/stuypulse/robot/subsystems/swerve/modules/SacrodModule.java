package com.stuypulse.robot.subsystems.swerve.modules;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;

import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Encoder;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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

    public SacrodModule(String name, int driveID, int turnID, int canCoderID, Translation2d location, Angle angleOffset, boolean driveInverted, boolean turnInverted) {
        this.name = name;
        this.location = location;
        this.angleOffset = angleOffset;

        driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        SparkBaseConfig driveConfig = new SparkMaxConfig().inverted(driveInverted).idleMode(IdleMode.kBrake);
        driveConfig.encoder.positionConversionFactor(Encoder.Drive.POSITION_CONVERSION).velocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder();

        turnMotor = new SparkMax(turnID, MotorType.kBrushless);
        SparkBaseConfig turnConfig = new SparkMaxConfig().inverted(turnInverted).idleMode(IdleMode.kBrake);
        turnConfig.encoder.positionConversionFactor(Encoder.Turn.POSITION_CONVERSION).velocityConversionFactor(Encoder.Turn.VELOCITY_CONVERSION);
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.driveController = new PIDController(Drive.kP, Drive.kP, Drive.kD)
                                .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());
        this.turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD);

        absoluteEncoder = new CANcoder(canCoderID);

        targetState = new SwerveModuleState();
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