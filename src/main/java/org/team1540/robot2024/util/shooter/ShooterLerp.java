package org.team1540.robot2024.util.shooter;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterLerp {
    InterpolatingDoubleTreeMap angle = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap left = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap right = new InterpolatingDoubleTreeMap();

    public void put(Double key, ShooterSetpoint value){
        angle.put(key, value.pivot.getDegrees());
        left.put(key, value.leftSetpoint);
        right.put(key, value.rightSetpoint);
    }

    @SafeVarargs
    public final ShooterLerp put(Pair<Double, ShooterSetpoint>... dataPoints){
        for (Pair<Double, ShooterSetpoint> point : dataPoints) {
            put(point.getFirst(), point.getSecond());
        }
        return this;
    }


    public ShooterSetpoint get(Double key){
        return new ShooterSetpoint(Rotation2d.fromDegrees(angle.get(key)), left.get(key), right.get(key));
    }
}
