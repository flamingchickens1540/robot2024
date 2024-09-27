package org.team1540.robot2024.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.team1540.robot2024.util.vision.LimelightHelpers;

import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AprilTagVisionIOMegaTag2 implements AprilTagVisionIO {
    private final String name;
    private final Supplier<Rotation2d> heading;
    private final DoubleSupplier headingVelocityRadPerSec;

    public AprilTagVisionIOMegaTag2(String name,
                                    Pose3d cameraOffsetMeters,
                                    Supplier<Rotation2d> heading,
                                    DoubleSupplier headingVelocityRadPerSec) {
        this.name = name;
        this.heading = heading;
        this.headingVelocityRadPerSec = headingVelocityRadPerSec;
        LimelightHelpers.setLEDMode_PipelineControl(name);
        setPoseOffset(cameraOffsetMeters);
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        LimelightHelpers.SetRobotOrientation(
                name,
                heading.get().getDegrees(),
                Math.toDegrees(headingVelocityRadPerSec.getAsDouble()),
                0, 0, 0, 0);
        LimelightHelpers.PoseEstimate measurement =
                DriverStation.isDisabled()
                        ? LimelightHelpers.getBotPoseEstimate_wpiBlue(name)
                        : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        Pose3d limelightPose = new Pose3d(measurement.pose);
        inputs.estimatedPoseMeters = new Pose3d(
                limelightPose.getX(),
                limelightPose.getY() + 0.105,
                limelightPose.getZ(),
                limelightPose.getRotation());
        inputs.lastMeasurementTimestampSecs = measurement.timestampSeconds;
        inputs.numTagsSeen = measurement.tagCount;
        inputs.seenTagIDs = Arrays.stream(measurement.rawFiducials).mapToInt(fiducial -> fiducial.id).toArray();
        inputs.avgTagDistance = measurement.avgTagDist;
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

    @Override
    public void takeSnapshot(String snapshotName) {
        LimelightHelpers.takeSnapshot(name, snapshotName);
    }
}
