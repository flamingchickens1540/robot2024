package org.team1540.robot2024.util.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;

public class AprilTagsCrescendo {
    private static AprilTagsCrescendo instance;
    private static AprilTagFieldLayout tags;
    public enum Tags {
        SOURCE_INNER(1, 10),
        SOURCE_OUTER(2, 9),
        SPEAKER_OFFSET(3, 8),
        SPEAKER_CENTER(4, 7),
        AMP(5,6),
        CLIMB_SOURCE(11,16),
        CLIMB_AMP(12,15),
        CLIMB_FAR(13,14);

        final int blue;
        final int red;
        private Tags(int red, int blue){
            this.red = red;
            this.blue = blue;
        }
    }

    public static AprilTagsCrescendo getInstance() {
        if (instance == null){
            instance = new AprilTagsCrescendo();
        }
        return instance;
    }

    private AprilTagsCrescendo(){
        tags = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }

    public static AprilTagFieldLayout getTags() {
        return tags;
    }

    public static Pose3d getTag(int tagID){
        return tags.getTagPose(tagID).get();
    }
    public static Pose3d getTag(Tags tag){
        return getTag(tag, DriverStation.getAlliance().orElse(null));
    }
    public static Pose3d getTag(Tags tag, DriverStation.Alliance alliance){
        return getTag(alliance == DriverStation.Alliance.Red ? tag.red : tag.blue);
    }
}
