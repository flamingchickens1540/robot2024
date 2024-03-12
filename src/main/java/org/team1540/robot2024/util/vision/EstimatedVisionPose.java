package org.team1540.robot2024.util.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.team1540.robot2024.Constants;

public class EstimatedVisionPose {
    public double timestampSecs = -1;
    public Pose3d poseMeters = new Pose3d();
    public int[] tagIDs = new int[0];
    public Pose3d[] tagPoses = new Pose3d[0];

    public double getAvgTagDistance() {
        double avgDistance = 0;
        for (Pose3d pose : tagPoses) avgDistance += pose.getTranslation().getNorm();
        avgDistance /= tagPoses.length;
        return avgDistance;
    }

    public Matrix<N3, N1> getStdDevs() {
        double avgDistance = getAvgTagDistance();

        double xyStdDev =
                Constants.Vision.XY_STD_DEV_COEFF
                    * Math.pow(avgDistance, 2.0)
                    / tagPoses.length;
        double rotStdDev =
                Constants.Vision.ROT_STD_DEV_COEFF
                    * Math.pow(avgDistance, 2.0)
                    / tagPoses.length;
        return VecBuilder.fill(xyStdDev, xyStdDev, tagIDs.length > 1 ? rotStdDev : Double.POSITIVE_INFINITY);
    }
}
