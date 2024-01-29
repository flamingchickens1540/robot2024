package org.team1540.robot2024.util.vision;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.Arrays;
import java.util.Objects;

/**
 * @param timestampSecs  RIO FPGA timestamp of this vision pose
 * @param poseMeters     Measured, blue alliance-origin pose from vision
 * @param seenTagIDs     Array of all tags used to compute this measurement
 * @param tagPosesMeters Pose in robot-space of all tags used to compute this measurement
 */
public record TimestampedVisionPose(
        double timestampSecs,
        Pose2d poseMeters,
        int[] seenTagIDs,
        Pose2d[] tagPosesMeters) {

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
