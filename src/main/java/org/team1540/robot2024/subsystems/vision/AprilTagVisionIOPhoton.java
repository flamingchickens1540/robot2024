package org.team1540.robot2024.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
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
import java.util.function.Supplier;

public class AprilTagVisionIOPhoton implements AprilTagVisionIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private final Supplier<Pose2d> drivetrainPoseSupplier;

    private Transform3d cameraTransform;
    private Pose3d lastEstimatedPose;

    public AprilTagVisionIOPhoton(String name, Pose3d cameraOffsetMeters, Supplier<Pose2d> drivetrainPoseSupplier) {
        camera = new PhotonCamera(name);
        cameraTransform = new Transform3d(cameraOffsetMeters.getTranslation(), cameraOffsetMeters.getRotation());
        photonEstimator = new PhotonPoseEstimator(
                AprilTagsCrescendo.getInstance().getTags(),
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera,
                cameraTransform);
        photonEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        lastEstimatedPose = new Pose3d();
        this.drivetrainPoseSupplier = drivetrainPoseSupplier;
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        photonEstimator.setReferencePose(drivetrainPoseSupplier.get());
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

            inputs.numTagsSeen = targets.size();
            if (inputs.numTagsSeen > 0) {
                inputs.avgTagDistance = 0;
                for (PhotonTrackedTarget target : targets)
                    inputs.avgTagDistance += new Pose3d().plus(target.getBestCameraToTarget()).getTranslation().getNorm();
                inputs.avgTagDistance /= inputs.numTagsSeen;
            }
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
