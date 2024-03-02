package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.shooter.ShooterSetpoint;

import static org.team1540.robot2024.Constants.Shooter.Pivot.HUB_SHOOT;

public class ShootSequence extends ParallelCommandGroup {

    public ShootSequence(Shooter shooter, Indexer indexer) {
        this(shooter, indexer, HUB_SHOOT);
    }

    public ShootSequence(Shooter shooter, Indexer indexer, ShooterSetpoint setpoint) {
        addCommands(

                new PrepareShooterCommand(shooter, setpoint),
                Commands.sequence(
                        Commands.sequence(
                                Commands.waitSeconds(1.5)
//                                Commands.waitUntil(() -> shooter.isPivotAtSetpoint() && shooter.areFlywheelsSpunUp())
                        ), //.withTimeout(1.5),

                        new IntakeAndFeed(indexer, () -> 1, () -> 0.5).withTimeout(3)
                )

                // TODO: Add a wait for having completed the shot (steady then current spike/velocity dip and then back down?)
        );
    }


    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
