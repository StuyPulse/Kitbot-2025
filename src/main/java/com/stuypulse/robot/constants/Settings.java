package com.stuypulse.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive.ModulePosition;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    double DT = 1.0 / 50.0;

    double WIDTH = Units.inchesToMeters(0); // intake side 
    double LENGTH = Units.inchesToMeters(0);

    public interface Dropper {
        double DROP_SPEED = 0.5;
        double REVERSE_SPEED = 0.5;

        double DROP_TIME = 0.75;
        double WAIT_TIME_AT_CORAL_STATION = 0.75;
    }
    
    public interface Swerve {
        double WIDTH = Units.inchesToMeters(22.984); // intake side 
        double LENGTH = Units.inchesToMeters(22.984); 

        double MAX_LINEAR_VELOCITY = 4.9;
        double MAX_LINEAR_ACCEL = 15;
        double MAX_ANGULAR_VELOCITY = 6.75; // (rad/s)
        double MAX_ANGULAR_ACCEL = 200.0; // (rad/s^2)

        double MODULE_VELOCITY_DEADBAND = 0.02; // (m/s)
        double MAX_MODULE_SPEED = 5.0; // (m/s)

        PathConstraints DEFAULT_CONSTRAINTS =
            new PathConstraints(
                MAX_LINEAR_VELOCITY,
                MAX_LINEAR_ACCEL,
                MAX_ANGULAR_VELOCITY,
                MAX_ANGULAR_ACCEL);
        
        public interface Alignment {
            SmartNumber X_TOLERANCE = new SmartNumber("Alignment/X Tolerance (m)", 0.1);
            SmartNumber Y_TOLERANCE = new SmartNumber("Alignment/Y Tolerance (m)", 0.1);
            SmartNumber THETA_TOLERANCE = new SmartNumber("Alignment/Theta Tolerance (rad)", 0.1);

            double XY_DEBOUNCE = 0.1;
            double THETA_DEBOUNCE = 0.1;
        }

        public interface ModuleOffsets {
            public static Translation2d getXYOffset(ModulePosition position) {
                switch (position) {
                    case FRONT_LEFT:
                        return FrontLeft.XY_OFFSET;
                    case FRONT_RIGHT:
                        return FrontRight.XY_OFFSET;
                    case BACK_LEFT:
                        return BackLeft.XY_OFFSET;
                    case BACK_RIGHT:
                        return BackRight.XY_OFFSET;
                    default:
                        throw new IllegalArgumentException("Not a valid swerve module position");
                }
            }
            public static Angle getAngleOffset(ModulePosition position) {
                switch (position) {
                    case FRONT_LEFT:
                        return FrontLeft.ANGLE_OFFSET;
                    case FRONT_RIGHT:
                        return FrontRight.ANGLE_OFFSET;
                    case BACK_LEFT:
                        return BackLeft.ANGLE_OFFSET;
                    case BACK_RIGHT:
                        return BackRight.ANGLE_OFFSET;
                    default:
                        throw new IllegalArgumentException("Not a valid swerve module position");
                }
            }
            public interface FrontLeft {
                Angle ANGLE_OFFSET = Angle.fromDegrees(0.0);
                Translation2d XY_OFFSET = new Translation2d(LENGTH * +0.5, WIDTH * +0.5);
            }
            public interface FrontRight {
                Angle ANGLE_OFFSET = Angle.fromDegrees(0.0);
                Translation2d XY_OFFSET = new Translation2d(LENGTH * +0.5, WIDTH * -0.5);
            }
            public interface BackLeft {
                Angle ANGLE_OFFSET = Angle.fromDegrees(0.0);
                Translation2d XY_OFFSET = new Translation2d(LENGTH * -0.5, WIDTH * +0.5);
            }
            public interface BackRight {
                Angle ANGLE_OFFSET = Angle.fromDegrees(0.0);
                Translation2d XY_OFFSET = new Translation2d(LENGTH * -0.5, WIDTH * -0.5);
            }
        }
        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    
                public interface Stages {
                    // input / output
                    double FIRST = 16.0 / 48.0;
                    double SECOND = 28.0 / 16.0;
                    double THIRD = 15.0 / 60.0;
                }
    
                double GEAR_RATIO = Stages.FIRST * Stages.SECOND * Stages.THIRD;
    
                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
    
            public interface Turn {
                double GEAR_RATIO = 1.0 / 12.8;
                double POSITION_CONVERSION = GEAR_RATIO * 2 * Math.PI;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
        }
        public interface Controllers {
            public interface Chassis {
                public interface XY {
                    public static Controller getController() {
                        return new PIDController(kP, kI, kD);
                    }
                    public static PIDConstants getConstants() {
                        return new PIDConstants(kP.get(), kI.get(), kD.get());
                    }

                    SmartNumber kP = new SmartNumber("Swerve/Chassis/PID/XY/kP", 3.0);
                    SmartNumber kI = new SmartNumber("Swerve/Chassis/PID/XY/kI", 0.0);
                    SmartNumber kD = new SmartNumber("Swerve/Chassis/PID/XY/kD", 0.0);
                }
                public interface Theta {
                    public static Controller getController() {
                        return new PIDController(kP, kI, kD);
                    }
                    public static PIDConstants getConstants() {
                        return new PIDConstants(kP.get(), kI.get(), kD.get());
                    }
                    SmartNumber kP = new SmartNumber("Swerve/Chassis/PID/Theta/kP", 3.0);
                    SmartNumber kI = new SmartNumber("Swerve/Chassis/PID/Theta/kI", 0.0);
                    SmartNumber kD = new SmartNumber("Swerve/Chassis/PID/Theta/kD", 0.0);
                }
            }
            public interface Modules {
                public interface Turn {
                    SmartNumber kP = new SmartNumber("Swerve/Modules/PID/Turn/kP", 3.5);
                    SmartNumber kI = new SmartNumber("Swerve/Modules/PID/Turn/kI", 0.0);
                    SmartNumber kD = new SmartNumber("Swerve/Modules/PID/Turn/kD", 0.1);
                }
            
                public interface Drive {
                    SmartNumber kP = new SmartNumber("Swerve/Modules/PID/Drive/kP", 1.3);
                    SmartNumber kI = new SmartNumber("Swerve/Modules/PID/Drive/kI", 0.0);
                    SmartNumber kD = new SmartNumber("Swerve/Modules/PID/Drive/kD", 0.0);
            
                    SmartNumber kS = new SmartNumber("Swerve/Modules/PID/Drive/kS", 0.17335);
                    SmartNumber kV = new SmartNumber("Swerve/Modules/PID/Drive/kV", 2.7274);
                    SmartNumber kA = new SmartNumber("Swerve/Modules/PID/Drive/kA", 0.456);
                }
            }
        }
    }

    public interface Vision {        
        Vector<N3> STDDEVS = VecBuilder.fill(0.3, 0.3, Math.toRadians(30));
    }

    public interface Driver {
        public interface Drive {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Drive/Deadband", 0.03);

            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);

            SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Drive/Max Speed", Swerve.MAX_LINEAR_VELOCITY);
            SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Drive/Max Accleration", Swerve.MAX_LINEAR_ACCEL);
        }

        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Turn/Deadband", 0.03);

            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);

            SmartNumber MAX_TELEOP_TURN_SPEED = new SmartNumber("Driver Settings/Turn/Max Turn Speed (rad/s)", Swerve.MAX_ANGULAR_VELOCITY);
            SmartNumber MAX_TELEOP_TURN_ACCEL = new SmartNumber("Driver Settings/Turn/Max Turn Accel (rad/s^2)", Swerve.MAX_ANGULAR_ACCEL);
        }
    }
}
