package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Driver.Turn;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDriveAwayFromReef extends Command {

    private final SwerveDrive swerve;

    public SwerveDriveDriveAwayFromReef() {
        swerve = SwerveDrive.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Translation2d robot = Odometry.getInstance().getPose().getTranslation();
        Translation2d reefToRobot = robot.minus(Field.REEF_CENTER);
        Vector2D velocity = new Vector2D(reefToRobot).normalize().mul(1.5);
        swerve.drive(velocity, 0);
    }
}
