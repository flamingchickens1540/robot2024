package org.team1540.robot2024.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.util.vision.AprilTagsCrescendo;

import java.util.List;
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
        photonEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        lastEstimatedPose = new Pose3d();
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        PhotonPipelineResult latestResult = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = latestResult.getTargets();
        Optional<EstimatedRobotPose> estimatedPose = photonEstimator.update(latestResult);

        double maxAmbiguityRatio = 0;
        for (PhotonTrackedTarget target : targets)
            if (target.getPoseAmbiguity() > maxAmbiguityRatio) maxAmbiguityRatio = target.getPoseAmbiguity();

        if (estimatedPose.isPresent() && maxAmbiguityRatio < Constants.Vision.MAX_AMBIGUITY_RATIO) {
            lastEstimatedPose = estimatedPose.get().estimatedPose;
            inputs.estimatedPoseMeters = lastEstimatedPose;
            inputs.lastMeasurementTimestampSecs = estimatedPose.get().timestampSeconds;
        }

        inputs.seenTagIDs = new int[targets.size()];
        inputs.tagPosesMeters = new Pose3d[targets.size()];
        for (int i = 0; i < targets.size(); i++) {
            inputs.seenTagIDs[i] = targets.get(i).getFiducialId();
            inputs.tagPosesMeters[i] = new Pose3d().plus(targets.get(i).getBestCameraToTarget());
        }
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
