package org.team1540.robot2024.util.vision;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class FlipUtil {
    public static Pose2d flipIfRed(Pose2d pose){
        return DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red
                ? GeometryUtil.flipFieldPose(pose)
                : pose;
    }

    public static Rotation2d flipIfRed(Rotation2d rotation){
        return DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red
                ? GeometryUtil.flipFieldRotation(rotation)
                : rotation;

    }

    public static double flipIfRed(double num){
        return DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red
                ? num *= -1
                : num;
    }
}
