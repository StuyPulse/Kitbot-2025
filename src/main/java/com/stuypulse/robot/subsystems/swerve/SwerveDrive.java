package com.stuypulse.robot.subsystems.swerve;

import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.modules.SacrodModule;
import com.stuypulse.robot.subsystems.swerve.modules.SimModule;
import com.stuypulse.robot.subsystems.swerve.modules.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

public class SwerveDrive extends SubsystemBase {

    private static final SwerveDrive instance;

    public enum ModulePosition {
        FRONT_RIGHT,
        FRONT_LEFT,
        BACK_LEFT,
        BACK_RIGHT
    }

    static {
        if (RobotBase.isReal()) {
            instance = new SwerveDrive(
                new SacrodModule(ModulePosition.FRONT_LEFT),
                new SacrodModule(ModulePosition.BACK_LEFT),
                new SacrodModule(ModulePosition.BACK_RIGHT),
                new SacrodModule(ModulePosition.FRONT_RIGHT)
            );
        } else {
            instance = new SwerveDrive(
                new SimModule(ModulePosition.FRONT_LEFT),
                new SimModule(ModulePosition.BACK_LEFT),
                new SimModule(ModulePosition.BACK_RIGHT),
                new SimModule(ModulePosition.FRONT_RIGHT)
            );
        }
    }

    public static SwerveDrive getInstance() {
        return instance;
    }

    /** MODULES */
    private final SwerveModule[] modules;

    /** SENSORS */
    private final AHRS gyro;

    private final SwerveDriveKinematics kinematics;

    private final FieldObject2d[] module2ds;

    /** PATHPLANNER */

    protected SwerveDrive(SwerveModule... modules) {
        this.modules = modules;

        gyro = new AHRS(SPI.Port.kMXP);

        kinematics = new SwerveDriveKinematics(getModuleOffsets());

        module2ds = new FieldObject2d[modules.length];
    }

    public void configureAutoBuilder() {
        try{
            AutoBuilder.configure(
                Odometry.getInstance()::getPose,
                (pose) -> Odometry.getInstance().reset(pose),
                this::getChassisSpeeds,
                (speeds, feedforwards) -> setChassisSpeeds(speeds),
                new PPHolonomicDriveController(
                    Settings.Swerve.Controllers.Chassis.XY.getConstants(), // Translation PID constants
                    Settings.Swerve.Controllers.Chassis.Theta.getConstants() // Rotation PID constants
                ),
                RobotConfig.fromGUISettings(),
                () -> false,
                instance
            );

            // PathPlannerLogging.setLogActivePathCallback((poses) -> Odometry.getInstance().getField().getObject("path").setPoses(poses));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void initFieldObjects(Field2d field) {
        for (int i = 0; i < modules.length; i++) {
            module2ds[i] = field.getObject(modules[i].getName()+"-2d");
            module2ds[i].setPose(Robot.isBlue() ? module2ds[i].getPose() : Field.transformToOppositeAlliance(module2ds[i].getPose()));
        }
    }

    private Translation2d[] getModuleOffsets() {
        Translation2d[] locations = new Translation2d[modules.length];

        for(int i = 0; i < modules.length; ++i) {
            locations[i] = modules[i].getOffset();
        }

        return locations;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for(int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for(int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    private SwerveModule getModule(String id) {
        for (SwerveModule module : modules)
            if (module.getName().equals(id)) {
                return module;
        }
        throw new IllegalArgumentException("Couldn't find the module with id \"" + id + "\"");
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getModuleStates());
    }

    /** MODULE STATES API **/
    public void drive(Vector2D velocity, double omega) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                velocity.x,
                velocity.y,
                -omega,
                Odometry.getInstance().getRotation());

        Pose2d robotVel = new Pose2d(
            Settings.DT * speeds.vxMetersPerSecond,
            Settings.DT * speeds.vyMetersPerSecond,
            Rotation2d.fromRadians(Settings.DT * speeds.omegaRadiansPerSecond));
        Twist2d twistVel = new Pose2d().log(robotVel);

        setChassisSpeeds(new ChassisSpeeds(
            twistVel.dx / Settings.DT,
            twistVel.dy / Settings.DT,
            twistVel.dtheta / Settings.DT
        ));
    }

    public void setChassisSpeeds(ChassisSpeeds robotSpeed) {
        setModuleStates(kinematics.toSwerveModuleStates(robotSpeed));
    }

    private static SwerveModuleState filterModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) > Swerve.MODULE_VELOCITY_DEADBAND)
            return state;

        return new SwerveModuleState(0, state.angle);
    }

    public void setModuleStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException("Number of desired module states does not match number of modules (" + modules.length + ")");
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.MAX_MODULE_SPEED);

        for(int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(filterModuleState(states[i]));
        }
    }

    public void stop() {
        setChassisSpeeds(new ChassisSpeeds());
    }

    /** GYRO API **/
    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }

    public double getGyroYaw() {
        return gyro.getYaw();
    }

    public double getGyroPitch() {
        return gyro.getPitch();
    }

    public double getGyroRoll() {
        return gyro.getRoll();
    }

    /** KINEMATICS **/
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void setXMode() {
        SwerveModuleState[] state = {
            new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        };
        setModuleStates(state);
    }

    @Override
    public void periodic() {
        Odometry odometry = Odometry.getInstance();
        Pose2d pose = odometry.getPose();
        Rotation2d angle = odometry.getRotation();

        for (int i = 0; i < modules.length; ++i) {
            Pose2d modulePose = new Pose2d(
                pose.getTranslation().plus(modules[i].getOffset().rotateBy(angle)),
                modules[i].getState().angle.plus(angle)
            );
            module2ds[i].setPose(Robot.isBlue() ? modulePose : Field.transformToOppositeAlliance(modulePose));
        }

        SmartDashboard.putNumber("Swerve/Gyro Angle (deg)", getGyroPitch());
        SmartDashboard.putNumber("Swerve/Gyro Pitch (deg)", getGyroPitch());
        SmartDashboard.putNumber("Swerve/Gyro Roll", getGyroRoll());

        SmartDashboard.putNumber("Swerve/X Acceleration (Gs)", gyro.getWorldLinearAccelX());
        SmartDashboard.putNumber("Swerve/Y Acceleration (Gs)", gyro.getWorldLinearAccelY());
        SmartDashboard.putNumber("Swerve/Z Acceleration (Gs)", gyro.getWorldLinearAccelZ());
    }

    @Override
    public void simulationPeriodic() {
        // Integrate omega in simulation and store in gyro
        var speeds = getKinematics().toChassisSpeeds(getModuleStates());

        gyro.setAngleAdjustment(gyro.getAngle() - Math.toDegrees(speeds.omegaRadiansPerSecond * Settings.DT));
    }

}