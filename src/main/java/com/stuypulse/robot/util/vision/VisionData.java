package com.stuypulse.robot.util.vision;

import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose2d;

/** This class stores pieces of data from the vision system. */
public class VisionData {

    private final Pose2d outputPose;
    private final int[] ids;
    private final double timestamp;

    public VisionData(Pose2d outputPose, int[] ids, double timestamp) {
        this.outputPose = outputPose;
        this.ids = ids;
        this.timestamp = timestamp;
    }

    /**
     * Returns the pose of the robot relative to the field.
     *
     * @return the pose of the robot relative to the field
     */
    public Pose2d getPose() {
        return outputPose;
    }

    /**
     * Returns the IDs of the tags detected.
     *
     * @return the IDs of the tags detected
     */
    public int[] getIDs() {
        return ids;
    }

    /**
     * Returns the timestamp of the vision data.
     *
     * @return the timestamp of the vision data
     */
    public double getTimestamp() {
        return timestamp;
    }

    /**
     * Returns if the data is valid.
     *
     * @return if valid data
     */
    public boolean isValidData() {
        for (long id : ids) {
            boolean found = false;
            for (AprilTag tag : Field.APRILTAGS) {
                if (tag.getID() == id) {
                    found = true;
                    break;
                }
            }

            if (!found) {
                return false;
            }
        }

        if (Double.isNaN(outputPose.getX())
                || outputPose.getX() < 0
                || outputPose.getX() > Field.LENGTH)
            return false;
        if (Double.isNaN(outputPose.getY())
                || outputPose.getY() < 0
                || outputPose.getY() > Field.WIDTH)
            return false;

        return true;
    }
}