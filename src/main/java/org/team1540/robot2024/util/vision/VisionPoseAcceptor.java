package org.team1540.robot2024.util.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Supplier;

import static org.team1540.robot2024.Constants.Vision.*;

public class VisionPoseAcceptor {
    private final Supplier<ChassisSpeeds> robotVelocitySupplier;
    private final Supplier<Double> elevatorVelocitySupplier;

    public VisionPoseAcceptor(
            Supplier<ChassisSpeeds> robotVelocitySupplier,
            Supplier<Double> elevatorVelocitySupplier) {
        this.robotVelocitySupplier = robotVelocitySupplier;
        this.elevatorVelocitySupplier = elevatorVelocitySupplier;
    }

    public boolean shouldAcceptVision(TimestampedVisionPose visionPose) {
        ChassisSpeeds robotVelocity = robotVelocitySupplier.get();
        double elevatorVelocity = elevatorVelocitySupplier.get();
        // Do not accept poses that have too much delay
        if (Timer.getFPGATimestamp() - visionPose.timestampSecs() >= MAX_VISION_DELAY_SECS) return false;

        // Do not accept poses that see too little tags
        if (visionPose.getNumTagsSeen() < MIN_ACCEPTED_NUM_TAGS) return false;

        // Do not accept poses that have an average tag distance that is too far away
        if (visionPose.getAverageTagDistance() > MAX_ACCEPTED_AVG_TAG_DIST_METERS) return false;

        // Do not accept poses taken when the robot has too much rotational or translational velocity
        boolean rotatingTooFast = robotVelocity.omegaRadiansPerSecond > MAX_ACCEPTED_ROT_SPEED_RAD_PER_SEC;
        boolean translatingTooFast =
                Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond)
                        > MAX_ACCEPTED_LINEAR_SPEED_MPS;
        boolean elevatorTooFast = Math.abs(elevatorVelocity) > MAX_ACCEPTED_ELEVATOR_SPEED_MPS;
        return !rotatingTooFast && !translatingTooFast && !elevatorTooFast;
    }
}
