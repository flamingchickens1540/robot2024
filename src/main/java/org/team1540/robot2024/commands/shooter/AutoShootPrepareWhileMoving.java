package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.commands.drivetrain.DriveWithSpeakerLookAheadCommand;
import org.team1540.robot2024.commands.drivetrain.DriveWithSpeakerTargetingCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.vision.AprilTagsCrescendo;

import java.util.function.Supplier;

import static org.team1540.robot2024.Constants.Targeting.SPEAKER_CENTER_HEIGHT;

public class AutoShootPrepareWhileMoving extends ParallelCommandGroup {
    public AutoShootPrepareWhileMoving(XboxController controller, Drivetrain drivetrain, Shooter shooter) {
        Pose3d speakerPose3d = new Pose3d(
                AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.SPEAKER_CENTER).toPose2d().getX(),
                AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.SPEAKER_CENTER).toPose2d().getY(),
                SPEAKER_CENTER_HEIGHT,
                new Rotation3d());
        Supplier<Pose2d> modifiedPosition = ()->{
            double flightDistance = speakerPose3d.getTranslation().getDistance(new Translation3d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), Constants.Shooter.Pivot.PIVOT_HEIGHT));
            double NOTE_VELOCITY = 32;
            double time = flightDistance/NOTE_VELOCITY;
            Translation2d modifier = new Translation2d(drivetrain.getChassisSpeeds().vxMetersPerSecond * time,drivetrain.getChassisSpeeds().vyMetersPerSecond*time);
            Pose2d drivetrainPose = drivetrain.getPose().plus(new Transform2d(modifier.getX(), modifier.getY(), new Rotation2d()));
            return drivetrainPose;
        };

        addCommands(
                new DriveWithSpeakerLookAheadCommand(drivetrain,controller),
                new AutoShootPrepare(modifiedPosition, shooter)
        );
    }
}
