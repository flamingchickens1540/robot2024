package org.team1540.robot2024.util.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.team1540.robot2024.Constants;

public class ShooterSetpoint {
    public final Rotation2d pivot;
    public final double leftSetpoint;
    public final double rightSetpoint;

    public ShooterSetpoint(Rotation2d pivot, double leftSetpoint, double rightSetpoint) {
        this.pivot = pivot;
        this.leftSetpoint = leftSetpoint;
        this.rightSetpoint = rightSetpoint;
    }
    public ShooterSetpoint(double pivotRots, double leftSetpoint, double rightSetpoint) {
        this(Rotation2d.fromRotations(pivotRots), leftSetpoint, rightSetpoint);
    }

    public ShooterSetpoint(Rotation2d pivot){
        this(pivot, Constants.Shooter.Flywheels.LEFT_RPM, Constants.Shooter.Flywheels.RIGHT_RPM);
    }
}
