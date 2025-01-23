package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Driver.Turn;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDrive extends Command {

    private final SwerveDrive swerve;

    private final Gamepad driver;

    private final VStream speed;
    private final IStream turn;

    public SwerveDriveDrive(Gamepad driver) {
        swerve = SwerveDrive.getInstance();

        speed = VStream.create(this::getDriverInputAsVelocity)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> x.pow(Drive.POWER.get()),
                x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL),
                new VLowPassFilter(Drive.RC));

        turn = IStream.create(driver::getRightX)
            .filtered(
                x -> SLMath.deadband(x, Turn.DEADBAND.get()),
                x -> SLMath.spow(x, Turn.POWER.get()),
                x -> x * Turn.MAX_TELEOP_TURN_SPEED.get(),
                new LowPassFilter(Turn.RC));

        this.driver = driver;

        addRequirements(swerve);
    }

    private Vector2D getDriverInputAsVelocity() {
        return new Vector2D(driver.getLeftStick().y, -driver.getLeftStick().x);
    }

    @Override
    public void execute() {
        Vector2D targetVelocity = speed.get();
        double omega = turn.get();

        ChassisSpeeds currentSpeed = swerve.getChassisSpeeds();
        double currentSpeedMagnitude = Math.hypot(currentSpeed.vxMetersPerSecond, currentSpeed.vyMetersPerSecond);

        // if (targetVelocity.magnitude() > 0.05 || Math.abs(omega) > 0.05 || currentSpeedMagnitude > 0.05 || Math.abs(currentSpeed.omegaRadiansPerSecond) > 0.05) {
        //     swerve.drive(speed.get(), turn.get());
        // }
        // else {
        //     swerve.setXMode();
        // }
        swerve.drive(speed.get(), turn.get());
    }
}
