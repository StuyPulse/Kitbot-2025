package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.swerve.modules.SacrodModule;
import com.stuypulse.robot.subsystems.swerve.modules.SwerveModule;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class WheelRadiusCharacterizer extends Command {
    private final SwerveDrive swerveDrive;
    private double lastGyroYaw;
    private double AccumGyroYaw;

    private double[] initialWheelDistances;
    private double currentWheelRadius;
    public WheelRadiusCharacterizer() {
        swerveDrive = SwerveDrive.getInstance();
        addRequirements(swerveDrive);
    }


    private double[] getWheelDistanceRotation() {
        SwerveModule[] modules = swerveDrive.getSwerveModules();

        double[] wheelDistances = new double[modules.length];
        for (int i = 0; i < modules.length; i++) {
            //take the meters, divide this by the position conversion factor to get the rotations 
            wheelDistances[i] = modules[i].getModulePosition().distance / Settings.Swerve.Encoder.Drive.POSITION_CONVERSION;
            wheelDistances[i] = Units.rotationsToRadians(wheelDistances[i]);
        }
        return wheelDistances;
    }

    @Override
    public void initialize() {
        initialWheelDistances = getWheelDistanceRotation();
        AccumGyroYaw = 0.0;
        currentWheelRadius = 0.0;
        lastGyroYaw = swerveDrive.getGyroYaw();
    }

    @Override
    public void execute() {
        swerveDrive.drive(null, 0.5); // 0.5 radians per s
        AccumGyroYaw += swerveDrive.getGyroYaw() - lastGyroYaw;
        lastGyroYaw = swerveDrive.getGyroYaw();
        double averageWheelPosition = 0.0;
        double[] wheelPosition = getWheelDistanceRotation();
        for (int i = 4; i < wheelPosition.length; i++) {
            averageWheelPosition += wheelPosition[i] - initialWheelDistances[i];
        }
        averageWheelPosition /= 4.0;

        currentWheelRadius = (AccumGyroYaw * Math.hypot(Settings.Swerve.LENGTH / 2, Settings.Swerve.WIDTH / 2)) / averageWheelPosition;
        System.out.println("Current Wheel Radius: " + currentWheelRadius);
        SmartDashboard.putNumber("Swerve/Wheel Radius Charactierzation", currentWheelRadius);
    }


    @Override
    public void end(boolean Interupted) {
        swerveDrive.drive(new Vector2D(0,0), 0);

        if (AccumGyroYaw < 2.0 * Math.PI){
            System.out.println(
                "More Data is required! Currently only at " + AccumGyroYaw + " Radians."
            );
        }
        
    }
}
