package org.team1540.robot2024.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import org.team1540.robot2024.util.vision.LimelightHelpers;

import java.util.Arrays;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {
    private final String name;
    private final NetworkTable table;
    private long lastHeartbeat;
    private double lastHeartbeatChangeTime;

    public AprilTagVisionIOLimelight(String name, Pose3d cameraOffsetMeters) {
        this.name = name;
        this.table = NetworkTableInstance.getDefault().getTable(name);
        LimelightHelpers.setCameraMode_Processor(name);
        LimelightHelpers.setLEDMode_PipelineControl(name);
        setPoseOffset(cameraOffsetMeters);
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        LimelightHelpers.PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (measurement.tagCount > 1) {
            Pose3d limelightPose = LimelightHelpers.getBotPose3d_wpiBlue(name);
            inputs.estimatedPoseMeters = new Pose3d(
                    limelightPose.getX(),
                    limelightPose.getY() + 0.105,
                    limelightPose.getZ(),
                    limelightPose.getRotation());
            inputs.lastMeasurementTimestampSecs = measurement.timestampSeconds;
        }
        inputs.numTagsSeen = measurement.tagCount;
        inputs.seenTagIDs = Arrays.stream(measurement.rawFiducials).mapToInt(fiducial -> fiducial.id).toArray();
        inputs.avgTagDistance = measurement.avgTagDist;

        long hb = table.getEntry("hb").getInteger(0);
        double currentTime = Timer.getFPGATimestamp();
        if (hb > lastHeartbeat) {
            lastHeartbeatChangeTime = currentTime;
            lastHeartbeat = hb;
        }
        inputs.connected = currentTime - lastHeartbeatChangeTime < 0.5;
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
