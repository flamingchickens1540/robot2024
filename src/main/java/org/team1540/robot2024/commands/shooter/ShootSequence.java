package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.commands.indexer.PrepareFeederForShooter;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;

public class ShootSequence extends SequentialCommandGroup {
    public ShootSequence(Shooter shooter, Indexer indexer) {
        addCommands(
                Commands.parallel(
                        new PrepareShooterCommand(shooter),
                        Commands.sequence(
                                indexer.moveNoteOut(),
                                new PrepareFeederForShooter(indexer)
                        )
                ),
                Commands.runOnce(() -> indexer.setIntakePercent(1), indexer)
                // TODO: Add a wait for having completed the shot (steady then current spike/velocity dip and then back down?)
        );
    }


    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
