/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive.ModulePosition;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.ControllerGroup;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
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
        double SCORE_SPEED = 0.1;
    }
    
    public interface Swerve {
        double WIDTH = 0.0; // intake side 
        double LENGTH = 0.0; 

        double MAX_LINEAR_VELOCITY = 4.9;
        double MAX_LINEAR_ACCEL = 15;
        double MAX_ANGULAR_VELOCITY = 6.75; // (rad/s)
        double MAX_ANGULAR_ACCEL = 200.0; // (rad/s^2)

        double MODULE_VELOCITY_DEADBAND = 0.02; // (m/s)
        double MAX_MODULE_SPEED = 5.0; // (m/s)

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
                    double kP = 0.0;
                    double kI = 0.0;
                    double kD = 0.0;
                }
                public interface Theta {
                    double kP = 0.0;
                    double kI = 0.0;
                    double kD = 0.0;
                }
            }
            public interface Modules {
                public static AnglePIDController getTurnController(ModulePosition position) {
                    switch (position) {
                        case FRONT_LEFT:
                            return new AnglePIDController(FrontLeft.Turn.kP, FrontLeft.Turn.kI, FrontLeft.Turn.kD);
                        case FRONT_RIGHT:
                            return new AnglePIDController(FrontRight.Turn.kP, FrontRight.Turn.kI, FrontRight.Turn.kD);
                        case BACK_LEFT:
                            return new AnglePIDController(BackLeft.Turn.kP, BackLeft.Turn.kI, BackLeft.Turn.kD);
                        case BACK_RIGHT:
                            return new AnglePIDController(BackRight.Turn.kP, BackRight.Turn.kI, BackRight.Turn.kD);
                        default:
                            throw new IllegalArgumentException();
                    }
                }
                public static ControllerGroup getDriveController(ModulePosition position) {
                    switch (position) {
                        case FRONT_LEFT:
                            return new PIDController(FrontLeft.Drive.kP, FrontLeft.Drive.kI, FrontLeft.Drive.kD)
                                .add(new MotorFeedforward(FrontLeft.Drive.kS, FrontLeft.Drive.kV, FrontLeft.Drive.kA).velocity());
                        case FRONT_RIGHT:
                            return new PIDController(FrontRight.Drive.kP, FrontRight.Drive.kI, FrontRight.Drive.kD)
                                .add(new MotorFeedforward(FrontRight.Drive.kS, FrontRight.Drive.kV, FrontRight.Drive.kA).velocity());
                        case BACK_LEFT:
                            return new PIDController(BackLeft.Drive.kP, BackLeft.Drive.kI, BackLeft.Drive.kD)
                                .add(new MotorFeedforward(BackLeft.Drive.kS, BackLeft.Drive.kV, BackLeft.Drive.kA).velocity());
                        case BACK_RIGHT:
                            return new PIDController(BackRight.Drive.kP, BackRight.Drive.kI, BackRight.Drive.kD)
                                .add(new MotorFeedforward(BackRight.Drive.kS, BackRight.Drive.kV, BackRight.Drive.kA).velocity());
                        default:
                            throw new IllegalArgumentException();
                    }
                }
                public interface FrontLeft {
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
                public interface FrontRight {
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
                public interface BackLeft {
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
                public interface BackRight {
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
    }

    public interface Vision {
        double POSE_AMBIGUITY_RATIO_THRESHOLD = 0.60;
        Vector<N3> STDDEVS = VecBuilder.fill(0.3, 0.3, Math.toRadians(30));
    }
}
