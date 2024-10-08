package org.team1540.robot2024.subsystems.vision.apriltag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import static org.team1540.robot2024.Constants.Vision.AprilTag.*;

public class AprilTagVisionIOSim implements AprilTagVisionIO {
    private final VisionSystemSim visionSystemSim;
    private final PhotonCameraSim cameraSim;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;

    private final Supplier<Pose2d> poseSupplier;

    private Transform3d cameraTransform;
    private Pose3d lastEstimatedPose;

    public AprilTagVisionIOSim(String name, Pose3d cameraOffsetMeters, Supplier<Pose2d> poseSupplier) {
        this.visionSystemSim = new VisionSystemSim(name);
        this.poseSupplier = poseSupplier;

        AprilTagFieldLayout aprilTagLayout;
        try {
            aprilTagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        visionSystemSim.addAprilTags(aprilTagLayout);

        SimCameraProperties cameraProps = new SimCameraProperties();
        cameraProps.setCalibration(SIM_RES_WIDTH, SIM_RES_HEIGHT, SIM_DIAGONAL_FOV);
        cameraProps.setFPS(SIM_FPS);
        cameraProps.setAvgLatencyMs(SIM_AVG_LATENCY_MS);
        camera = new PhotonCamera(name);
        cameraSim = new PhotonCameraSim(camera, cameraProps);

        cameraTransform = new Transform3d(cameraOffsetMeters.getTranslation(), cameraOffsetMeters.getRotation());
        visionSystemSim.addCamera(cameraSim, cameraTransform);

        photonEstimator = new PhotonPoseEstimator(
                aprilTagLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera,
                cameraTransform);
        photonEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        lastEstimatedPose = new Pose3d(poseSupplier.get());
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        visionSystemSim.update(poseSupplier.get());

        photonEstimator.setReferencePose(poseSupplier.get());
        PhotonPipelineResult latestResult = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = latestResult.getTargets();
        Optional<EstimatedRobotPose> estimatedPose = photonEstimator.update(latestResult);

        double maxAmbiguityRatio = 0;
        for (PhotonTrackedTarget target : targets)
            if (target.getPoseAmbiguity() > maxAmbiguityRatio) maxAmbiguityRatio = target.getPoseAmbiguity();

        if (estimatedPose.isPresent() && maxAmbiguityRatio < MAX_AMBIGUITY_RATIO) {
            lastEstimatedPose = estimatedPose.get().estimatedPose;
            inputs.estimatedPoseMeters = lastEstimatedPose;
            inputs.lastMeasurementTimestampSecs = estimatedPose.get().timestampSeconds;

            inputs.numTagsSeen = targets.size();
            inputs.seenTagIDs = targets.stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray();
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
        visionSystemSim.adjustCamera(cameraSim, cameraTransform);
    }

    @Override
    public String getName() {
        return camera.getName();
    }
}
