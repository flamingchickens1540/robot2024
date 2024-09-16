package org.team1540.robot2024.subsystems.vision.gamepiece;

import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

public interface NoteVisionIO {
    @AutoLog
    class NoteVisionIOInputs {
        public boolean hasDetection = false;
        public double lastDetectionTimestampSecs = 0.0;
        public Rotation3d targetRotation = new Rotation3d(); // Camera-relative target rotation
        public double targetArea = 0.0;
        public String targetClass = "";
    }

    default void updateInputs(NoteVisionIOInputs inputs) {}

    default String getName() {
        return "";
    }
}
