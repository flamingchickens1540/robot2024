package org.team1540.robot2024.util.vision;

import edu.wpi.first.math.geometry.Rotation3d;

public record GamepieceDetection(
        double timestampSecs,
        Rotation3d rotation, // Robot relative gamepiece rotation
        double area,
        String targetClass) {}
