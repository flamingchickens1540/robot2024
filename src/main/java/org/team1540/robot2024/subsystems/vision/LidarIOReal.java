package org.team1540.robot2024.subsystems.vision;

import org.team1540.robot2024.Constants;
import org.team1540.robot2024.util.LIDARLite;

public class LidarIOReal implements LidarIO{
    private final LIDARLite lidar = new LIDARLite(Constants.Vision.LIDAR_PORT);

    @Override
    public void updateInputs(LidarIOInputs inputs) {
        inputs.distanceMeters = lidar.getDistance();
    }

    
}
