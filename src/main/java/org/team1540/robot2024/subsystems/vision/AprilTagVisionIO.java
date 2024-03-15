package org.team1540.robot2024.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface AprilTagVisionIO {
    @AutoLog
    class AprilTagVisionIOInputs {
        public Pose3d estimatedPoseMeters = new Pose3d();
        public int numTagsSeen = 0;
        public double avgTagDistance = Double.POSITIVE_INFINITY;
        public double lastMeasurementTimestampSecs = 0.0;
    }

    default void updateInputs(AprilTagVisionIOInputs inputs) {}

    /** Sets the robot-space pose of the camera */
    default void setPoseOffset(Pose3d newPose) {}

    default String getName() {
        return "";
    }
}
