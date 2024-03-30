package org.team1540.robot2024.subsystems.vision;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import org.team1540.robot2024.util.vision.LimelightHelpers;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {
    private final String name;

    public AprilTagVisionIOLimelight(String name, Pose3d cameraOffsetMeters) {
        this.name = name;
        LimelightHelpers.setCameraMode_Processor(name);
        LimelightHelpers.setLEDMode_PipelineControl(name);
        setPoseOffset(cameraOffsetMeters);
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        LimelightHelpers.PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (measurement.tagCount > 1) {
            inputs.estimatedPoseMeters =
                    LimelightHelpers.getBotPose3d_wpiBlue(name)
                            .transformBy(new Transform3d(0, 0.105, 0, new Rotation3d()));
            inputs.lastMeasurementTimestampSecs = measurement.timestampSeconds;
        }
        inputs.numTagsSeen = measurement.tagCount;
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
