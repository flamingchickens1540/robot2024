package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.commands.drivetrain.DriveWithSpeakerTargetingCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.shooter.ShooterSetpoint;
import org.team1540.robot2024.util.vision.AprilTagsCrescendo;

public class AutoShootPrepare extends ParallelCommandGroup {
    public AutoShootPrepare(XboxController controller, Drivetrain drivetrain, Shooter shooter) {
        addCommands(
                new DriveWithSpeakerTargetingCommand(drivetrain,controller),
                new PrepareShooterCommand(shooter, () -> new ShooterSetpoint(
                        Rotation2d.fromRadians(
                                Math.atan2(Constants.Targeting.SPEAKER_CENTER_HEIGHT - Constants.Shooter.Pivot.PIVOT_HEIGHT, drivetrain.getPose().getTranslation().getDistance(
                                        AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.SPEAKER_CENTER).toPose2d().getTranslation()
                                ))).minus(Constants.Shooter.Pivot.REAL_ZEROED_ANGLE),
                        8000, 6000
                ))
        );
    }
}
