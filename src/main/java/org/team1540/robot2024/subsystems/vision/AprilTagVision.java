package org.team1540.robot2024.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.util.vision.TimestampedVisionPose;
import org.team1540.robot2024.util.vision.VisionPoseAcceptor;

import java.util.Arrays;
import java.util.Optional;
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
    private final VisionPoseAcceptor poseAcceptor;

    private TimestampedVisionPose frontPose =
            new TimestampedVisionPose(-1, new Pose2d(), new int[0], new Pose2d[0], true, false);
    private TimestampedVisionPose rearPose =
            new TimestampedVisionPose(-1, new Pose2d(), new int[0], new Pose2d[0], false, true);

    public AprilTagVision(
            AprilTagVisionIO frontCameraIO,
            AprilTagVisionIO rearCameraIO,
            Consumer<TimestampedVisionPose> visionPoseConsumer,
            Supplier<Double> elevatorHeightSupplierMeters,
            VisionPoseAcceptor poseAcceptor) {
        this.frontCameraIO = frontCameraIO;
        this.rearCameraIO = rearCameraIO;
        this.visionPoseConsumer = visionPoseConsumer;
        this.elevatorHeightSupplierMeters = elevatorHeightSupplierMeters;
        this.poseAcceptor = poseAcceptor;
    }

    @Override
    public void periodic() {
        // TODO: need to change if one camera is stationary
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
                    frontCameraInputs.tagPosesMeters,
                    true,
                    false);
        }
        if (rearCameraInputs.lastMeasurementTimestampSecs > rearPose.timestampSecs()) {
            rearPose = new TimestampedVisionPose(
                    rearCameraInputs.lastMeasurementTimestampSecs,
                    rearCameraInputs.estimatedPoseMeters,
                    rearCameraInputs.seenTagIDs,
                    rearCameraInputs.tagPosesMeters,
                    false,
                    true);
        }

        Optional<TimestampedVisionPose> latestPose = getEstimatedPose();
        latestPose.ifPresent(visionPose -> Logger.recordOutput("Vision/EstimatedPose", visionPose.poseMeters()));
        latestPose.ifPresent(visionPoseConsumer);
    }

    /**
     * Gets the estimated pose by fusing individual computed poses from each camera.
     * Returns null if no tags seen, in simulation, or if the elevator is moving
     * too fast
     */
    public Optional<TimestampedVisionPose> getEstimatedPose() {
        boolean useFrontPose = poseAcceptor.shouldAcceptVision(frontPose);
        boolean useRearPose = poseAcceptor.shouldAcceptVision(rearPose);

        if (useFrontPose && useRearPose) {
            int[] allTagIDs = Arrays.copyOf(frontPose.seenTagIDs(), frontPose.getNumTagsSeen() + rearPose.getNumTagsSeen());
            System.arraycopy(rearPose.seenTagIDs(), 0, allTagIDs, frontPose.getNumTagsSeen(), rearPose.getNumTagsSeen());
            Pose2d[] allTagPoses = Arrays.copyOf(frontPose.tagPosesMeters(), frontPose.getNumTagsSeen() + rearPose.getNumTagsSeen());
            System.arraycopy(rearPose.tagPosesMeters(), 0, allTagPoses, frontPose.getNumTagsSeen(), rearPose.getNumTagsSeen());

            return Optional.of(new TimestampedVisionPose(
                    (frontPose.timestampSecs() + rearPose.timestampSecs()) / 2,
                    frontPose.poseMeters().interpolate(rearPose.poseMeters(), 0.5),
                    allTagIDs,
                    allTagPoses,
                    true,
                    true));
        } else if (useFrontPose) return Optional.of(frontPose);
        else if (useRearPose) return Optional.of(rearPose);
        else return Optional.empty();
    }
}
