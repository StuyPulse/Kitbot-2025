package com.stuypulse.robot.constants;

import com.pathplanner.lib.path.PathConstraints;
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

    double WIDTH = Units.inchesToMeters(36); // intake side 
    double LENGTH = Units.inchesToMeters(36);

    public interface Dropper {
        boolean inverted = true;
        
        double DROP_SPEED = 0.3;
        double REVERSE_SPEED = 0.4;

        double DROP_TIME = 0.75;
        double WAIT_TIME_AT_CORAL_STATION = 0.75;
    }
    
    public interface Swerve {
        double WIDTH = Units.inchesToMeters(22.213); // intake side 
        double LENGTH = Units.inchesToMeters(22.213); 

        double MAX_LINEAR_VELOCITY = 5.0;
        double MAX_LINEAR_ACCEL = 15.0;
        double MAX_ANGULAR_VELOCITY = 3.5; // (rad/s)
        double MAX_ANGULAR_ACCEL = 35.0; // (rad/s^2)

        double MODULE_VELOCITY_DEADBAND = 0.05; // (m/s)
        double MAX_MODULE_SPEED = 4.0; // (m/s)

        PathConstraints DEFAULT_CONSTRAINTS =
            new PathConstraints(
                MAX_LINEAR_VELOCITY,
                MAX_LINEAR_ACCEL,
                MAX_ANGULAR_VELOCITY,
                MAX_ANGULAR_ACCEL);
        
        public interface Alignment {
            public interface XY {
                SmartNumber kP = new SmartNumber("Swerve/Chassis/PID/XY/kP", 2.0);
                SmartNumber kI = new SmartNumber("Swerve/Chassis/PID/XY/kI", 0.0);
                SmartNumber kD = new SmartNumber("Swerve/Chassis/PID/XY/kD", 0.01);
            }

            public interface Theta {
                SmartNumber kP = new SmartNumber("Swerve/Chassis/PID/Theta/kP", 5.0);
                SmartNumber kI = new SmartNumber("Swerve/Chassis/PID/Theta/kI", 0.0);
                SmartNumber kD = new SmartNumber("Swerve/Chassis/PID/Theta/kD", 0.2);
            }

            SmartNumber X_TOLERANCE = new SmartNumber("Alignment/X Tolerance (m)", 0.05);
            SmartNumber Y_TOLERANCE = new SmartNumber("Alignment/Y Tolerance (m)", 0.05);
            SmartNumber THETA_TOLERANCE = new SmartNumber("Alignment/Theta Tolerance (rad)", 0.1);

            double XY_DEBOUNCE = 0.05;
            double THETA_DEBOUNCE = 0.05;
        }

        public interface Turn {
            SmartNumber kP = new SmartNumber("Swerve/Modules/PID/Turn/kP", 3.5);
            SmartNumber kI = new SmartNumber("Swerve/Modules/PID/Turn/kI", 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Modules/PID/Turn/kD", 0.1);
        }
    
        public interface Drive {
            SmartNumber kP = new SmartNumber("Swerve/Modules/PID/Drive/kP", 0.0);
            SmartNumber kI = new SmartNumber("Swerve/Modules/PID/Drive/kI", 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Modules/PID/Drive/kD", 0.01);
    
            SmartNumber kS = new SmartNumber("Swerve/Modules/PID/Drive/kS", 0.26722);
            SmartNumber kV = new SmartNumber("Swerve/Modules/PID/Drive/kV", 2.2119);
            SmartNumber kA = new SmartNumber("Swerve/Modules/PID/Drive/kA", 0.36249);
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

        public interface FrontLeft {
            boolean DRIVE_INVERTED = false;
            boolean TURN_INVERTED = false;
            Angle ANGLE_OFFSET = Angle.fromDegrees(24.785156);
            Translation2d XY_OFFSET = new Translation2d(LENGTH * +0.5, WIDTH * +0.5);
        }
        public interface FrontRight {
            boolean DRIVE_INVERTED = false;
            boolean TURN_INVERTED = false;
            Angle ANGLE_OFFSET = Angle.fromDegrees(-85.913086);
            Translation2d XY_OFFSET = new Translation2d(LENGTH * +0.5, WIDTH * -0.5);
        }
        public interface BackLeft {
            boolean DRIVE_INVERTED = false;
            boolean TURN_INVERTED = false;
            Angle ANGLE_OFFSET = Angle.fromDegrees(26.762695);
            Translation2d XY_OFFSET = new Translation2d(LENGTH * -0.5, WIDTH * +0.5);
        }
        public interface BackRight {
            boolean DRIVE_INVERTED = false;
            boolean TURN_INVERTED = false;
            Angle ANGLE_OFFSET = Angle.fromDegrees(-23.686523);
            Translation2d XY_OFFSET = new Translation2d(LENGTH * -0.5, WIDTH * -0.5);
        }
    }

    public interface Vision {
        SmartNumber DISTANCE_CUTOFF = new SmartNumber("Vision/Distance Cutoff", 5);
        Vector<N3> MIN_STDEV = VecBuilder.fill(0.3, 0.3, 5);
    }

    public interface Driver {
        public interface Drive {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Drive/Deadband", 0.05);

            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);

            SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Drive/Max Speed", Swerve.MAX_LINEAR_VELOCITY);
            SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Drive/Max Accleration", Swerve.MAX_LINEAR_ACCEL);
        }

        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Turn/Deadband", 0.05);

            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);

            SmartNumber MAX_TELEOP_TURN_SPEED = new SmartNumber("Driver Settings/Turn/Max Turn Speed (rad/s)", Swerve.MAX_ANGULAR_VELOCITY);
            SmartNumber MAX_TELEOP_TURN_ACCEL = new SmartNumber("Driver Settings/Turn/Max Turn Accel (rad/s^2)", Swerve.MAX_ANGULAR_ACCEL);
        }
    }
}
