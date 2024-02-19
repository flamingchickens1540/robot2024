package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.indexer.PrepareFeederForShooter;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;

public class ShootSequence extends SequentialCommandGroup {
    public ShootSequence(Shooter shooter, Indexer indexer) {
        addCommands(
                Commands.parallel(
                        new PrepareShooterCommand(shooter),
                        new WaitCommand(0.7),
                        Commands.sequence(
                                indexer.moveNoteOut(),
                                new PrepareFeederForShooter(indexer)
                        )
                ),
                new IntakeAndFeed(indexer, 1, 0.5),
//                Commands.waitUntil(() -> !indexer.isFeederAtSetpoint())
                new WaitCommand(15),
                new InstantCommand(() -> {
                    indexer.setFeederPercent(0);
                    indexer.setIntakePercent(0);
                    shooter.stopFlywheels();
                })
                // TODO: Add a wait for having completed the shot (steady then current spike/velocity dip and then back down?)
        );
    }


    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
