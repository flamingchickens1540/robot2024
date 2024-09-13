package org.team1540.robot2024.subsystems.vision.gamepiece;

import org.littletonrobotics.junction.AutoLog;

public interface NoteVisionIO {
    @AutoLog
    class NoteVisionIOInputs {
        public boolean hasDetection = false;
        public double lastDetectionTimestampSecs = 0.0;
        public double targetPitchRads = 0.0;
        public double targetYawRads = 0.0;
        public double targetAreaRads = 0.0;
        public String targetClass = "";
    }

    default void updateInputs(NoteVisionIOInputs inputs) {}

    default String getName() {
        return "";
    }
}
