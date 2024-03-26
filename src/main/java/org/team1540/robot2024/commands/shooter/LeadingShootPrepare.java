package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.vision.AprilTagsCrescendo;

import java.util.function.Supplier;

import static org.team1540.robot2024.Constants.Targeting.SPEAKER_CENTER_HEIGHT;

public class LeadingShootPrepare extends SequentialCommandGroup {
    
    
    public LeadingShootPrepare(Drivetrain drivetrain, Shooter shooter, double leftSetpoint, double rightSetpoint){
        Supplier<Translation2d> modifiedPosition = ()->{
            Pose3d speakerPose3d = new Pose3d(
                    AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.SPEAKER_CENTER).toPose2d().getX(),
                    AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.SPEAKER_CENTER).toPose2d().getY(),
                    SPEAKER_CENTER_HEIGHT,
                    new Rotation3d());
            double flightDistance = speakerPose3d.getTranslation().getDistance(new Translation3d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), Constants.Shooter.Pivot.PIVOT_HEIGHT));
//            double NOTE_VELOCITY = 6000.0*1.5*2.0*Math.PI*2.54/100.0/60.0;
            double NOTE_VELOCITY = 12.18;
            double time = flightDistance/NOTE_VELOCITY;
            Translation2d modifier = new Translation2d(drivetrain.getChassisSpeeds().vxMetersPerSecond * time,drivetrain.getChassisSpeeds().vyMetersPerSecond*time);
            return modifier;

        };
        addCommands(
                new AutoShootPrepare(()->
                        drivetrain.getPose().plus(new Transform2d(modifiedPosition.get().getX(), modifiedPosition.get().getY(), new Rotation2d())),
                        shooter, leftSetpoint, rightSetpoint)
        );
        
    }
    
    public LeadingShootPrepare(Drivetrain drivetrain, Shooter shooter){
        this(drivetrain, shooter, 7000, 3000);
    }
}
