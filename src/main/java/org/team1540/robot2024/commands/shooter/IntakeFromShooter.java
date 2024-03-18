package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.shooter.ShooterSetpoint;

public class IntakeFromShooter extends SequentialCommandGroup {

    public IntakeFromShooter(Shooter shooter, Indexer indexer) {
        addCommands(
            Commands.deadline(
                    Commands.sequence(
                            new WaitUntilCommand(indexer::isNoteStaged),
                            new WaitUntilCommand(() -> !indexer.isNoteStaged())
                    ),
                    new IntakeAndFeed(indexer, () -> -1, () -> -1),
                    new PrepareShooterCommand(
                            shooter,
                            () -> new ShooterSetpoint(Rotation2d.fromDegrees(40), -2000, -2000) // Tune this
                    )
            ),
            new IntakeCommand(indexer, () -> false, 1)
        );
    }
}
