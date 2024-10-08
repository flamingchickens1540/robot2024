package org.team1540.robot2024.subsystems.vision.gamepiece;

import edu.wpi.first.math.geometry.Rotation3d;
import org.team1540.robot2024.util.vision.LimelightHelpers;

import static org.team1540.robot2024.Constants.Vision.Gamepiece.*;

public class NoteVisionIOLimelight implements NoteVisionIO {
    private final String name;

    public NoteVisionIOLimelight(String name) {
        this.name = name;
        LimelightHelpers.setLEDMode_PipelineControl(name);
        LimelightHelpers.setPipelineIndex(name, PIPELINE_INDEX);
    }

    @Override
    public void updateInputs(NoteVisionIOInputs inputs) {
        inputs.hasDetection = LimelightHelpers.getTV(name);
        if (inputs.hasDetection) {
            inputs.lastDetectionTimestampSecs =
                    (LimelightHelpers.getLimelightNTTableEntry(name, "tv").getLastChange() / 1000000.0)
                            - ((LimelightHelpers.getLatency_Capture(name) + LimelightHelpers.getLatency_Pipeline(name))
                            / 1000.0);
            //TODO check rotation signs
            inputs.targetRotation =
                    new Rotation3d(
                            0,
                            -Math.toRadians(LimelightHelpers.getTY(name)),
                            Math.toRadians(LimelightHelpers.getTX(name)));
            inputs.targetArea = LimelightHelpers.getTA(name);
            inputs.targetClass = LimelightHelpers.getNeuralClassID(name);
        }
    }

    @Override
    public String getName() {
        return name;
    }
}
