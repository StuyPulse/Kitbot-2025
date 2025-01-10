package com.stuypulse.robot.commands.swerve;

import java.util.function.Supplier;

import com.pathplanner.lib.config.PIDConstants;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings.Swerve.Controllers.Chassis.Theta;
import com.stuypulse.robot.constants.Settings.Swerve.Controllers.Chassis.XY;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwervePIDToPose extends Command {

    private final SwerveDrive swerve;
    private final Odometry odometry;

    private final HolonomicController controller;
    private final Supplier<Pose2d> poseSupplier;
    private final BStream isAligned;
    private final IStream velocityError;

    private final FieldObject2d targetPose2d;

    private Number xTolerance;
    private Number yTolerance;
    private Number thetaTolerance;
    private Number velocityTolerance;

    private Pose2d targetPose;

    public SwervePIDToPose(Pose2d targetPose) {
        this(() -> targetPose);
    }

    public SwervePIDToPose(Supplier<Pose2d> poseSupplier) {
        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();

        this.poseSupplier = poseSupplier;

        targetPose2d = Odometry.getInstance().getField().getObject("Target Pose");

        controller = new HolonomicController(
            new PIDController(XY.kP, XY.kI, XY.kD),
            new PIDController(XY.kP, XY.kI, XY.kD),
            new AnglePIDController(Theta.kP, Theta.kI, Theta.kD));

        isAligned = BStream.create(this::isAligned)
            .filtered(new BDebounceRC.Both(Alignment.THETA_DEBOUNCE));

        velocityError = IStream.create(() -> {
            ChassisSpeeds speeds = controller.getError();

            return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).getNorm();
        })
        .filtered(new LowPassFilter(0.05))
        .filtered(x -> Math.abs(x));

        xTolerance = Alignment.X_TOLERANCE;
        yTolerance = Alignment.Y_TOLERANCE;
        thetaTolerance = Alignment.THETA_TOLERANCE;
        velocityTolerance = 0.15;

        addRequirements(swerve);
    }

    public SwervePIDToPose withTranslationConstants(PIDConstants pid) {
        controller.setTranslationConstants(pid.kP, pid.kI, pid.kD);
        return this;
    }
    
    public SwervePIDToPose withRotationConstants(PIDConstants pid) {
        controller.setRotationConstants(pid.kP, pid.kI, pid.kD);
        return this;
    }

    public SwervePIDToPose withTranslationConstants(double p, double i, double d) {
        controller.setTranslationConstants(p, i, d);
        return this;
    }
    
    public SwervePIDToPose withRotationConstants(double p, double i, double d) {
        controller.setRotationConstants(p, i, d);
        return this;
    }

    public SwervePIDToPose withTolerance(Number x, Number y, Number theta) {
        xTolerance = x;
        yTolerance = y;
        thetaTolerance = theta;
        return this;
    }

    @Override
    public void initialize() {
        targetPose = poseSupplier.get();
    }

    private boolean isAligned() {
        return controller.isDone(xTolerance.doubleValue(), yTolerance.doubleValue(), Math.toDegrees(thetaTolerance.doubleValue()))
            && velocityError.get() < velocityTolerance.doubleValue();
    }

    @Override
    public void execute() {
        targetPose2d.setPose(targetPose);
        controller.update(targetPose, odometry.getPose());

        Vector2D speed = new Vector2D(controller.getOutput().vxMetersPerSecond, controller.getOutput().vyMetersPerSecond)
            .clamp(Swerve.MAX_LINEAR_VELOCITY);
        double rotation = SLMath.clamp(controller.getOutput().omegaRadiansPerSecond, Swerve.MAX_ANGULAR_VELOCITY);
        
        SmartDashboard.putNumber("Alignment/Translation Target Speed", speed.distance());

        if (Math.abs(rotation) < Swerve.Alignment.THETA_TOLERANCE.get())
            rotation = 0;

        ChassisSpeeds clamped = new ChassisSpeeds(
            speed.x, speed.y, rotation);
        
        swerve.setChassisSpeeds(clamped);
    }

    @Override
    public boolean isFinished() {
        return isAligned.get();
    }

    @Override
    public void end(boolean interrupted) {
        Field.clearFieldObject(targetPose2d);
    }

}
