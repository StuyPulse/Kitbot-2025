package com.stuypulse.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive.ModulePosition;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.ControllerGroup;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
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
        double DROP_SPEED = 0.1;
        double DROP_TIME = 1.0;
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
            double X_TOLERANCE = 0.1; // meters
            double Y_TOLERANCE = 0.1; 
            double THETA_TOLERANCE = 0.1; // ????

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
                        return new PIDConstants(kP, kI, kD);
                    }
                    double kP = 3.0;
                    double kI = 0.0;
                    double kD = 0.0;
                }
                public interface Theta {
                    public static Controller getController() {
                        return new PIDController(kP, kI, kD);
                    }
                    public static PIDConstants getConstants() {
                        return new PIDConstants(kP, kI, kD);
                    }
                    double kP = 3.0;
                    double kI = 0.0;
                    double kD = 0.0;
                }
            }
            public interface Modules {
                public interface Turn {
                    double kP = 3.5;
                    double kI = 0.0;
                    double kD = 0.1;
                }
            
                public interface Drive {
                    double kP = 1.3;
                    double kI = 0.0;
                    double kD = 0.0;
            
                    double kS = 0.17335;
                    double kV = 2.7274;
                    double kA = 0.456;
                }
            }
        }
    }

    public interface Vision {
        double POSE_AMBIGUITY_RATIO_THRESHOLD = 0.60;
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
