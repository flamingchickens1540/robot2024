package org.team1540.robot2024.util.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotState;

import static org.team1540.robot2024.Constants.Vision.AprilTag.*;

public class EstimatedVisionPose {
    public double timestampSecs = -1;
    public Pose3d poseMeters = new Pose3d();
    public int numTags = 0;
    public double avgDistance = Double.POSITIVE_INFINITY;

    public Matrix<N3, N1> getStdDevs() {

        double xyStdDev =
                XY_STD_DEV_COEFF
                    * Math.pow(avgDistance, 2.0)
                    / numTags;
        double rotStdDev =
                ROT_STD_DEV_COEFF
                    * Math.pow(avgDistance, 2.0)
                    / numTags;
        boolean acceptYaw = numTags > 1 || (numTags > 0 && RobotState.isDisabled());
        return VecBuilder.fill(xyStdDev, xyStdDev, acceptYaw ? rotStdDev : Double.POSITIVE_INFINITY);
    }
}
