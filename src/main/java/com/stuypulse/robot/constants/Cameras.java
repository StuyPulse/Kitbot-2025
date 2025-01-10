package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;

/** This interface stores information about each camera. */
public interface Cameras {

    public CameraInfo[] PhotonVisionCameras = new CameraInfo[] {
        new CameraInfo("camera 1 placeholder", new Pose3d()),
        new CameraInfo("camera 2 placeholder", new Pose3d()),
        new CameraInfo("camera 3 placeholder", new Pose3d()),
    };

    public CameraInfo[] LimelightCameras = new CameraInfo[] {
        new CameraInfo("camera 1 placeholder", new Pose3d()),
        new CameraInfo("camera 2 placeholder", new Pose3d()),
        new CameraInfo("camera 3 placeholder", new Pose3d()),
    };
    

    public static class CameraInfo {
        private String name;
        private Pose3d location;

        public CameraInfo(String name, Pose3d location) {
            this.name = name;
            this.location = location;
        }

        public String getName() {
            return name;
        }

        public Pose3d getLocation() {
            return location;
        }
    }
}