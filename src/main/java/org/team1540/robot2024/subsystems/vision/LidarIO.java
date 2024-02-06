package org.team1540.robot2024.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface LidarIO {

    @AutoLog
    class LidarIOInputs {
        public double distanceMeters = 0.0;
    }

    default void updateInputs(LidarIOInputs inputs) {}

}
