/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.swerve.modules;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Controllers.Modules;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive.ModulePosition;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimModule extends SwerveModule {

    private static LinearSystem<N2, N1, N2> identifyVelocityPositionSystem(double kV, double kA) {
        if (kV <= 0.0) {
            throw new IllegalArgumentException("Kv must be greater than zero.");
        }
        if (kA <= 0.0) {
            throw new IllegalArgumentException("Ka must be greater than zero.");
        }

        Matrix<N2, N2> systemMatrix = new Matrix<>(Nat.N2(), Nat.N2());
        systemMatrix.set(0, 0, 0.0);
        systemMatrix.set(0, 1, 1.0);
        systemMatrix.set(1, 0, 0.0);
        systemMatrix.set(1, 1, -kV / kA);

        Matrix<N2, N1> inputMatrix = new Matrix<>(Nat.N2(), Nat.N1());
        inputMatrix.set(0, 0, 0.0);
        inputMatrix.set(1, 0, 1.0 / kA);

        Matrix<N2, N2> outputMatrix = new Matrix<>(Nat.N2(), Nat.N2());
        outputMatrix.set(0, 0, 1.0);
        outputMatrix.set(0, 1, 0.0);
        outputMatrix.set(1, 0, 0.0);
        outputMatrix.set(1, 1, 1.0);

        Matrix<N2, N1> feedthroughMatrix = new Matrix<>(Nat.N2(), Nat.N1());
        feedthroughMatrix.fill(0.0);

        return new LinearSystem<N2, N1, N2>(systemMatrix, inputMatrix, outputMatrix, feedthroughMatrix);
    }

    // module data
    private final String name;
    private final Translation2d location;
    private SwerveModuleState targetState;

    // turn
    private final LinearSystemSim<N2, N1, N2> turnSim;

    // drive
    private final LinearSystemSim<N2, N1, N2> driveSim;

    // controllers
    private Controller driveController;
    private AngleController turnController;

    public SimModule(ModulePosition position) {

        // module data
        this.name = position.toString();
        this.location = Settings.Swerve.ModuleOffsets.getXYOffset(position);

        this.driveController = new PIDController(Modules.Drive.kP, Modules.Drive.kI, Modules.Drive.kD);
        this.turnController = new AnglePIDController(Modules.Turn.kP, Modules.Turn.kI, Modules.Turn.kD);

        targetState = new SwerveModuleState();

        turnSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(0.25, 0.007));

        driveSim = new LinearSystemSim<>(identifyVelocityPositionSystem(Modules.Drive.kV, Modules.Drive.kA));
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
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    private double getVelocity() {
        return driveSim.getOutput(1);
    }

    private double getDistance() {
        return driveSim.getOutput(0);
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRadians(turnSim.getOutput(0));
    }

    @Override
    public void setTargetState(SwerveModuleState state) {
        state.optimize(getAngle());
        targetState = state;
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }

    @Override
    public void periodic() {
        // turn
        turnController.update(
            Angle.fromRotation2d(targetState.angle),
            Angle.fromRotation2d(getAngle()));

        // drive
        driveController.update(
            targetState.speedMetersPerSecond,
            getVelocity());

        SmartDashboard.putNumber("Swerve/" + name + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/" + name + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/" + name + "/Angle Error", turnController.getError().toDegrees());
        SmartDashboard.putNumber("Swerve/" + name + "/Angle Voltage", turnController.getOutput());
        SmartDashboard.putNumber("Swerve/" + name + "/Target Velocity", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/" + name + "/Velocity", getVelocity());
        SmartDashboard.putNumber("Swerve/" + name + "/Velocity Error", driveController.getError());
        SmartDashboard.putNumber("Swerve/" + name + "/Velocity Voltage", driveController.getOutput());
    }

    @Override
    public void simulationPeriodic() {
        // drive
        driveSim.setInput(driveController.getOutput());
        driveSim.update(Settings.DT);

        // turn
        turnSim.setInput(turnController.getOutput());
        turnSim.update(Settings.DT);
    }
}