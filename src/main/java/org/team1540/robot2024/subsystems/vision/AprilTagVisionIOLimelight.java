package org.team1540.robot2024.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {
    private final String name;
    private final NetworkTable table;

    public AprilTagVisionIOLimelight(String name, Pose3d cameraOffsetMeters) {
        this.name = name;
        this.table = NetworkTableInstance.getDefault().getTable(name);
        table.getEntry("camMode").setNumber(0);
        table.getEntry("ledMode").setNumber(0);
        setPoseOffset(cameraOffsetMeters);
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        double[] botPose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        inputs.estimatedPoseMeters = new Pose3d(
                botPose[0],
                botPose[1],
                botPose[2],
                new Rotation3d(
                        Math.toRadians(botPose[3]),
                        Math.toRadians(botPose[4]),
                        Math.toRadians(botPose[5])));

        boolean hasTargets = table.getEntry("tv").getNumber(0).intValue() == 1;
        inputs.seenTagIDs = hasTargets ? new int[]{table.getEntry("tid").getNumber(0).intValue()} : new int[0];
        double[] tagPose = table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        inputs.tagPosesMeters = hasTargets
                ? new Pose3d[] {new Pose3d(
                        tagPose[0],
                        tagPose[1],
                        tagPose[2],
                        new Rotation3d(
                                Math.toRadians(tagPose[3]),
                                Math.toRadians(tagPose[4]),
                                Math.toRadians(tagPose[5])))}
                : new Pose3d[0];

        inputs.lastMeasurementTimestampSecs = Timer.getFPGATimestamp()
                - (table.getEntry("cl").getDouble(0) + table.getEntry("tl").getDouble(0)) / 1000.0;
    }

    @Override
    public void setPoseOffset(Pose3d poseOffsetMeters) {
        table.getEntry("camerapose_robotspace_set").setDoubleArray(new double[]{
                poseOffsetMeters.getX(),
                poseOffsetMeters.getY(),
                poseOffsetMeters.getZ(),
                Math.toDegrees(poseOffsetMeters.getRotation().getX()),
                Math.toDegrees(poseOffsetMeters.getRotation().getY()),
                Math.toDegrees(poseOffsetMeters.getRotation().getZ())
        });
    }

    @Override
    public String getName() {
        return name;
    }
}
