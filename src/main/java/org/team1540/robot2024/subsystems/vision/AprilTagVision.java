package org.team1540.robot2024.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.util.vision.EstimatedVisionPose;

import java.util.function.Consumer;
import java.util.function.Supplier;

import static org.team1540.robot2024.Constants.Vision.*;

public class AprilTagVision extends SubsystemBase {
    private final AprilTagVisionIO frontCameraIO;
    private final AprilTagVisionIOInputsAutoLogged frontCameraInputs = new AprilTagVisionIOInputsAutoLogged();

    private final AprilTagVisionIO rearCameraIO;
    private final AprilTagVisionIOInputsAutoLogged rearCameraInputs = new AprilTagVisionIOInputsAutoLogged();

    private final Consumer<EstimatedVisionPose> visionPoseConsumer;
    private final Supplier<Double> elevatorHeightSupplierMeters;

    private final EstimatedVisionPose frontPose = new EstimatedVisionPose();
    private final EstimatedVisionPose rearPose = new EstimatedVisionPose();

    private static boolean hasInstance = false;

    private AprilTagVision(
            AprilTagVisionIO frontCameraIO,
            AprilTagVisionIO rearCameraIO,
            Consumer<EstimatedVisionPose> visionPoseConsumer,
            Supplier<Double> elevatorHeightSupplierMeters) {
        if (hasInstance) throw new IllegalStateException("Instance of vision already exists");
        hasInstance = true;

        this.frontCameraIO = frontCameraIO;
        this.rearCameraIO = rearCameraIO;
        this.visionPoseConsumer = visionPoseConsumer;
        this.elevatorHeightSupplierMeters = elevatorHeightSupplierMeters;
    }

    public static AprilTagVision createReal(Consumer<EstimatedVisionPose> visionPoseConsumer,
                                            Supplier<Double> elevatorHeightSupplierMeters,
                                            Supplier<Pose2d> drivetrainPoseSupplier) {
        if (Constants.currentMode != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real vision on simulated robot", false);
        }
        return new AprilTagVision(
                new AprilTagVisionIOPhoton(FRONT_CAMERA_NAME, FRONT_CAMERA_POSE, drivetrainPoseSupplier),
                new AprilTagVisionIOPhoton(REAR_CAMERA_NAME, REAR_CAMERA_POSE, drivetrainPoseSupplier),
                visionPoseConsumer,
                elevatorHeightSupplierMeters);
    }

    public static AprilTagVision createSim(Consumer<EstimatedVisionPose> visionPoseConsumer,
                                           Supplier<Pose2d> drivetrainPoseSupplier,
                                           Supplier<Double> elevatorHeightSupplierMeters) {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated vision on real robot", false);
        }
        return new AprilTagVision(
                new AprilTagVisionIOSim(FRONT_CAMERA_NAME, FRONT_CAMERA_POSE, drivetrainPoseSupplier),
                new AprilTagVisionIOSim(REAR_CAMERA_NAME, REAR_CAMERA_POSE, drivetrainPoseSupplier),
                visionPoseConsumer,
                elevatorHeightSupplierMeters);
    }

    public static AprilTagVision createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy vision on real robot", false);
        }
        return new AprilTagVision(
                new AprilTagVisionIO() {},
                new AprilTagVisionIO() {},
                (pose) -> {},
                () -> 0.0);
    }

    @Override
    public void periodic() {
        frontCameraIO.setPoseOffset(
                new Pose3d(
                        FRONT_CAMERA_POSE.getX(),
                        FRONT_CAMERA_POSE.getY(),
                        FRONT_CAMERA_POSE.getZ() + elevatorHeightSupplierMeters.get(),
                        FRONT_CAMERA_POSE.getRotation())
        );
        rearCameraIO.setPoseOffset(
                new Pose3d(
                        REAR_CAMERA_POSE.getX(),
                        REAR_CAMERA_POSE.getY(),
                        REAR_CAMERA_POSE.getZ() + elevatorHeightSupplierMeters.get(),
                        REAR_CAMERA_POSE.getRotation())
        );

        frontCameraIO.updateInputs(frontCameraInputs);
        rearCameraIO.updateInputs(rearCameraInputs);
        Logger.processInputs("Vision/FrontCamera", frontCameraInputs);
        Logger.processInputs("Vision/RearCamera", rearCameraInputs);

        updateAndAcceptPose(frontCameraInputs, frontPose);
        updateAndAcceptPose(rearCameraInputs, rearPose);
    }

    private void updateAndAcceptPose(AprilTagVisionIOInputsAutoLogged cameraInputs, EstimatedVisionPose pose) {
        if (cameraInputs.lastMeasurementTimestampSecs > pose.timestampSecs) {
            pose.timestampSecs = cameraInputs.lastMeasurementTimestampSecs;
            pose.poseMeters = cameraInputs.estimatedPoseMeters;
            pose.tagIDs = cameraInputs.seenTagIDs;
            pose.tagPoses = cameraInputs.tagPosesMeters;
            visionPoseConsumer.accept(pose);
        }
    }
}
