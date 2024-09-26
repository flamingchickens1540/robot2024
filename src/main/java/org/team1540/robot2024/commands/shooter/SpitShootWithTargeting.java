package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.commands.drivetrain.DriveWithTargetingCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;

public class SpitShootWithTargeting extends ParallelCommandGroup {
    public SpitShootWithTargeting(XboxController controller, Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        addCommands(
                new DriveWithTargetingCommand(drivetrain,controller, Constants.Targeting::getSpeakerPose),
                new SpitShoot(shooter, indexer)
        );
    }
}
