package org.team1540.robot2024.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface AprilTagVisionIO {
    @AutoLog
    class AprilTagVisionIOInputs {
        public Pose2d estimatedPoseMeters = new Pose2d();
        public boolean hasTarget = false;
        public int[] seenTagIDs = {};
        public Pose2d[] tagPosesMeters = {};
        public double lastMeasurementTimestampSecs = 0.0;
    }

    default void updateInputs(AprilTagVisionIOInputs inputs) {}

    default String getName() {
        return "";
    }
}
