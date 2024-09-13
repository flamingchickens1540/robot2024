package org.team1540.robot2024.subsystems.vision.gamepiece;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.util.vision.GamepieceDetection;

import static org.team1540.robot2024.Constants.Vision.Gamepiece.*;

public class NoteVision extends SubsystemBase {
    private final NoteVisionIO io;
    private final NoteVisionIOInputsAutoLogged inputs = new NoteVisionIOInputsAutoLogged();

    private static boolean hasInstance = false;

    private NoteVision(NoteVisionIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of NoteVision already exists");
        this.io = io;
        hasInstance = true;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public GamepieceDetection getLatestDetection() {
        return new GamepieceDetection(
                inputs.lastDetectionTimestampSecs,
                inputs.targetPitchRads,
                inputs.targetYawRads,
                inputs.targetAreaRads,
                inputs.targetClass);
    }

    public boolean hasDetection() {
        return inputs.hasDetection;
    }

    public static NoteVision createReal() {
        return new NoteVision(new NoteVisionIOLimelight(CAMERA_NAME));
    }

    public static NoteVision createSim() {
        return createDummy();
    }

    public static NoteVision createDummy() {
        return new NoteVision(new NoteVisionIO() {});
    }
}
