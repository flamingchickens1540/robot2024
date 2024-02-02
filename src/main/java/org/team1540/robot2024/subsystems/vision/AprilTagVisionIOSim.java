package org.team1540.robot2024.subsystems.vision;

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
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import static org.team1540.robot2024.Constants.Vision.*;

public class AprilTagVisionIOSim implements AprilTagVisionIO {
    private final VisionSystemSim visionSystemSim;
    private final PhotonCameraSim cameraSim;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;

    private final Supplier<Pose2d> poseSupplier;

    private Transform3d cameraTransform;
    private Pose2d lastEstimatedPose;

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
        photonEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE);
        lastEstimatedPose = poseSupplier.get();
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        photonEstimator.setReferencePose(lastEstimatedPose);
        visionSystemSim.update(poseSupplier.get());

        Optional<EstimatedRobotPose> estimatedPose = photonEstimator.update();
        List<PhotonTrackedTarget> trackedTargets = camera.getLatestResult().getTargets();

        if (estimatedPose.isPresent()) {
            lastEstimatedPose = estimatedPose.get().estimatedPose.toPose2d();
            inputs.estimatedPoseMeters = lastEstimatedPose;
            inputs.lastMeasurementTimestampSecs = estimatedPose.get().timestampSeconds;
        }
        inputs.seenTagIDs = new int[trackedTargets.size()];
        inputs.tagPosesMeters = new Pose2d[trackedTargets.size()];
        for (int i = 0; i < trackedTargets.size(); i++) {
            PhotonTrackedTarget target = trackedTargets.get(i);
            inputs.seenTagIDs[i] = target.getFiducialId();
            inputs.tagPosesMeters[i] =
                    new Pose3d().plus(target.getBestCameraToTarget().plus(cameraTransform.inverse())).toPose2d();
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
