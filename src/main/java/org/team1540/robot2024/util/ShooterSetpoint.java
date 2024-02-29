package org.team1540.robot2024.util;

import edu.wpi.first.math.geometry.Rotation2d;

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
        this.pivot = Rotation2d.fromRotations(pivotRots);
        this.leftSetpoint = leftSetpoint;
        this.rightSetpoint = rightSetpoint;
    }
}
