package org.team1540.robot2024.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.team1540.robot2024.util.vision.AprilTagsCrescendo;

import java.util.Optional;

public class AprilTagVisionIOPhoton implements AprilTagVisionIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;

    private Transform3d cameraTransform;
    private Pose3d lastEstimatedPose;

    public AprilTagVisionIOPhoton(String name, Pose3d cameraOffsetMeters) {
        camera = new PhotonCamera(name);
        cameraTransform = new Transform3d(cameraOffsetMeters.getTranslation(), cameraOffsetMeters.getRotation());
        photonEstimator = new PhotonPoseEstimator(
                AprilTagsCrescendo.getInstance().getTags(),
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera,
                cameraTransform);
        photonEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE);
        lastEstimatedPose = new Pose3d();
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        Optional<EstimatedRobotPose> estimatedPose = photonEstimator.update();
        PhotonPipelineResult latestResult = camera.getLatestResult();

        if (estimatedPose.isPresent()) {
            lastEstimatedPose = estimatedPose.get().estimatedPose;
            inputs.estimatedPoseMeters = lastEstimatedPose;
            inputs.lastMeasurementTimestampSecs = estimatedPose.get().timestampSeconds;
        }

        inputs.hasTargets = latestResult.hasTargets();
        inputs.primaryTagID = inputs.hasTargets ? latestResult.getBestTarget().getFiducialId() : -1;
        inputs.primaryTagPoseMeters =
                inputs.hasTargets
                        ? new Pose3d().plus(
                        latestResult.getBestTarget().getBestCameraToTarget().plus(cameraTransform.inverse()))
                        : new Pose3d();
    }

    @Override
    public void setPoseOffset(Pose3d newPose) {
        cameraTransform = new Transform3d(newPose.getTranslation(), newPose.getRotation());
        photonEstimator.setRobotToCameraTransform(cameraTransform);
    }

    @Override
    public String getName() {
        return camera.getName();
    }
}
