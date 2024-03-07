package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.indexer.PrepareFeederForShooter;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;

public class ShootSequence extends ParallelCommandGroup {
    public ShootSequence(Shooter shooter, Indexer indexer) {
        addCommands(
                new PrepareShooterCommand(shooter),
                Commands.sequence(
                        Commands.sequence(
                                indexer.moveNoteOut(),
                                new PrepareFeederForShooter(indexer)
                        ).withTimeout(0.7), // TODO: 2/19/2024 fix this
                        new IntakeAndFeed(indexer, () -> 1, () -> 0.5).withTimeout(15)
                )
                // TODO: Add a wait for having completed the shot (steady then current spike/velocity dip and then back down?)
        );
    }


    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
