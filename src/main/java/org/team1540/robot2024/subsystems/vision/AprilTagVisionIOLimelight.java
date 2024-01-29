package org.team1540.robot2024.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.team1540.robot2024.util.vision.LimelightHelpers;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {
    private final String name;

    public AprilTagVisionIOLimelight(String name, Pose3d cameraOffsetMeters) {
        this.name = name;
        LimelightHelpers.setCameraPose_RobotSpace(
                name,
                cameraOffsetMeters.getX(),
                cameraOffsetMeters.getY(),
                cameraOffsetMeters.getZ(),
                Math.toDegrees(cameraOffsetMeters.getRotation().getX()),
                Math.toDegrees(cameraOffsetMeters.getRotation().getY()),
                Math.toDegrees(cameraOffsetMeters.getRotation().getZ()));
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        LimelightHelpers.Results results = LimelightHelpers.getLatestResults(name).targetingResults;
        LimelightHelpers.LimelightTarget_Fiducial[] fiducialTargets = results.targets_Fiducials;

        inputs.estimatedPoseMeters = new Pose3d(
                results.botpose_wpiblue[0],
                results.botpose_wpiblue[1],
                results.botpose_wpiblue[2],
                new Rotation3d(
                        Math.toRadians(results.botpose_wpiblue[3]),
                        Math.toRadians(results.botpose_wpiblue[4]),
                        Math.toRadians(results.botpose_wpiblue[5])
                )
        ).toPose2d();
        inputs.hasTarget = results.valid;
        inputs.seenTargetIDs = new int[fiducialTargets.length];
        for (int i = 0; i < fiducialTargets.length; i++) inputs.seenTargetIDs[i] = (int) fiducialTargets[i].fiducialID; // fiducial tag ids had better be ints
        inputs.targetPosesMeters = new Pose2d[fiducialTargets.length];
        for (int i = 0; i < fiducialTargets.length; i++) inputs.targetPosesMeters[i] = fiducialTargets[i].getTargetPose_RobotSpace2D();

        inputs.captureLatency = results.latency_capture;
        inputs.pipelineLatency = results.latency_pipeline;
    }

    @Override
    public String getName() {
        return name;
    }
}
