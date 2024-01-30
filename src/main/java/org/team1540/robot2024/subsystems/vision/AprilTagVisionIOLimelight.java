package org.team1540.robot2024.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import org.team1540.robot2024.util.vision.LimelightHelpers;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {
    private final String name;

    public AprilTagVisionIOLimelight(String name, Pose3d cameraOffsetMeters) {
        this.name = name;
        setPoseOffset(cameraOffsetMeters);
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

        inputs.seenTagIDs = new int[fiducialTargets.length];
        for (int i = 0; i < fiducialTargets.length; i++)
            inputs.seenTagIDs[i] = (int) fiducialTargets[i].fiducialID; // fiducial tag ids had better be ints

        inputs.tagPosesMeters = new Pose2d[fiducialTargets.length];
        for (int i = 0; i < fiducialTargets.length; i++)
            inputs.tagPosesMeters[i] = fiducialTargets[i].getTargetPose_RobotSpace2D();

        inputs.lastMeasurementTimestampSecs =
                Timer.getFPGATimestamp() - (results.latency_capture + results.latency_pipeline) / 1000.0;
    }

    @Override
    public void setPoseOffset(Pose3d poseOffsetMeters) {
        LimelightHelpers.setCameraPose_RobotSpace(
                name,
                poseOffsetMeters.getX(),
                poseOffsetMeters.getY(),
                poseOffsetMeters.getZ(),
                Math.toDegrees(poseOffsetMeters.getRotation().getX()),
                Math.toDegrees(poseOffsetMeters.getRotation().getY()),
                Math.toDegrees(poseOffsetMeters.getRotation().getZ()));
    }

    @Override
    public String getName() {
        return name;
    }
}
