/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    public interface Swerve {
        public interface Drive {
            int FRONT_LEFT = 0;
            int FRONT_RIGHT = 0;
            int BACK_LEFT = 0;
            int BACK_RIGHT = 0;
        }
        public interface Turn {
            int FRONT_LEFT = 0;
            int FRONT_RIGHT = 0;
            int BACK_LEFT = 0;
            int BACK_RIGHT = 0;
        }
        public interface CANCoderIds {
            int FRONT_LEFT = 0;
            int FRONT_RIGHT = 0;
            int BACK_LEFT = 0;
            int BACK_RIGHT = 0;
        }
    }

    public interface Dropper {
        int MOTOR = 0;
    }
}
