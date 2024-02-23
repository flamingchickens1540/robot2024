package org.team1540.robot2024.util.vision;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.Objects;

public class TimestampedVisionPose {
    public double timestampSecs = -1;
    public Pose2d poseMeters = new Pose2d();
    public boolean hasTargets = false;
    public int primaryTagID = -1;
    public Pose2d primaryTagPose = new Pose2d();

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        TimestampedVisionPose that = (TimestampedVisionPose) o;
        return Double.compare(timestampSecs, that.timestampSecs) == 0
                && hasTargets == that.hasTargets
                && primaryTagID == that.primaryTagID
                && Objects.equals(poseMeters, that.poseMeters)
                && Objects.equals(primaryTagPose, that.primaryTagPose);
    }
}
