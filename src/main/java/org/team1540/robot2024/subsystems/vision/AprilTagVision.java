package org.team1540.robot2024.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.util.vision.TimestampedVisionPose;

import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static org.team1540.robot2024.Constants.Vision.*;

public class AprilTagVision extends SubsystemBase {
    private final AprilTagVisionIO frontCameraIO;
    private final AprilTagVisionIOInputsAutoLogged frontCameraInputs = new AprilTagVisionIOInputsAutoLogged();

    private final AprilTagVisionIO rearCameraIO;
    private final AprilTagVisionIOInputsAutoLogged rearCameraInputs = new AprilTagVisionIOInputsAutoLogged();

    private final Consumer<TimestampedVisionPose> visionPoseConsumer;
    private final Supplier<Double> elevatorHeightSupplierMeters;

    private TimestampedVisionPose frontPose =
            new TimestampedVisionPose(-1, new Pose2d(), new int[0], new Pose2d[0]);
    private TimestampedVisionPose rearPose =
            new TimestampedVisionPose(-1, new Pose2d(), new int[0], new Pose2d[0]);

    public AprilTagVision(
            AprilTagVisionIO frontCameraIO,
            AprilTagVisionIO rearCameraIO,
            Consumer<TimestampedVisionPose> visionPoseConsumer,
            Supplier<Double> elevatorHeightSupplierMeters) {
        this.frontCameraIO = frontCameraIO;
        this.rearCameraIO = rearCameraIO;
        this.visionPoseConsumer = visionPoseConsumer;
        this.elevatorHeightSupplierMeters = elevatorHeightSupplierMeters;
    }

    @Override
    public void periodic() {
        frontCameraIO.setPoseOffset(
                new Pose3d(
                        FRONT_CAMERA_POSE.getX(),
                        FRONT_CAMERA_POSE.getY(),
                        FRONT_CAMERA_POSE.getZ() + elevatorHeightSupplierMeters.get(),
                        FRONT_CAMERA_POSE.getRotation())
        );
        rearCameraIO.setPoseOffset(
                new Pose3d(
                        REAR_CAMERA_POSE.getX(),
                        REAR_CAMERA_POSE.getY(),
                        REAR_CAMERA_POSE.getZ() + elevatorHeightSupplierMeters.get(),
                        REAR_CAMERA_POSE.getRotation())
        );

        frontCameraIO.updateInputs(frontCameraInputs);
        rearCameraIO.updateInputs(rearCameraInputs);
        Logger.processInputs("Vision/FrontCamera", frontCameraInputs);
        Logger.processInputs("Vision/RearCamera", rearCameraInputs);

        if (frontCameraInputs.lastMeasurementTimestampSecs > frontPose.timestampSecs()) {
            frontPose = new TimestampedVisionPose(
                    frontCameraInputs.lastMeasurementTimestampSecs,
                    frontCameraInputs.estimatedPoseMeters,
                    frontCameraInputs.seenTagIDs,
                    frontCameraInputs.tagPosesMeters);
        }
        if (rearCameraInputs.lastMeasurementTimestampSecs > rearPose.timestampSecs()) {
            rearPose = new TimestampedVisionPose(
                    rearCameraInputs.lastMeasurementTimestampSecs,
                    rearCameraInputs.estimatedPoseMeters,
                    rearCameraInputs.seenTagIDs,
                    rearCameraInputs.tagPosesMeters);
        }

        TimestampedVisionPose latestPose = getEstimatedPose();
        Logger.recordOutput("Vision/EstimatedPose", latestPose == null ? new Pose2d() : latestPose.poseMeters());
        visionPoseConsumer.accept(latestPose);
    }

    /**
     * Gets the estimated pose by fusing individual computed poses from each camera.
     * Returns null if no tags seen or in simulation
     */
    public TimestampedVisionPose getEstimatedPose() {
        if (Constants.currentMode == Constants.Mode.SIM) return null;
        if (frontPose.getNumTagsSeen() < 1 && rearPose.getNumTagsSeen() < 1) return null;
        else if (frontPose.getNumTagsSeen() < 1) return rearPose;
        else if (rearPose.getNumTagsSeen() < 1) return frontPose;

        // This just takes the average of the measurements, we could change this to something more advanced if necessary
        int[] allTagIDs = Arrays.copyOf(frontPose.seenTagIDs(), frontPose.getNumTagsSeen() + rearPose.getNumTagsSeen());
        System.arraycopy(rearPose.seenTagIDs(), 0, allTagIDs, frontPose.getNumTagsSeen(), rearPose.getNumTagsSeen());
        Pose2d[] allTagPoses = Arrays.copyOf(frontPose.tagPosesMeters(), frontPose.getNumTagsSeen() + rearPose.getNumTagsSeen());
        System.arraycopy(rearPose.tagPosesMeters(), 0, allTagPoses, frontPose.getNumTagsSeen(), rearPose.getNumTagsSeen());

        return new TimestampedVisionPose(
                (frontPose.timestampSecs() + rearPose.timestampSecs()) / 2,
                frontPose.poseMeters().interpolate(rearPose.poseMeters(), 0.5),
                allTagIDs,
                allTagPoses);
    }
}
