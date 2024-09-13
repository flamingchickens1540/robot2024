package org.team1540.robot2024.util.vision;

public record GamepieceDetection(
        double timestampSecs,
        double yawRads,
        double pitchRads,
        double areaRads,
        String targetClass) {}
