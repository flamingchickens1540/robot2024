package org.team1540.robot2024.util.vision;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.Arrays;
import java.util.Objects;

public class TimestampedVisionPose {
    public double timestampSecs = -1;
    public Pose2d poseMeters = new Pose2d();
    public int[] seenTagIDs = new int[0];
    public Pose2d[] tagPosesMeters = new Pose2d[0];

    public int getNumTagsSeen() {
        return seenTagIDs.length;
    }

    public double getAverageTagDistance() {
        double sumDistances = 0.0;
        for (Pose2d pose : tagPosesMeters) sumDistances += pose.getTranslation().getNorm();
        return sumDistances / getNumTagsSeen();
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        TimestampedVisionPose that = (TimestampedVisionPose) o;
        return Double.compare(timestampSecs, that.timestampSecs) == 0
                && Objects.equals(poseMeters, that.poseMeters)
                && Arrays.equals(seenTagIDs, that.seenTagIDs)
                && Arrays.equals(tagPosesMeters, that.tagPosesMeters);
    }
}
