package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.shooter.ShooterSetpoint;

import java.util.function.Supplier;

import static org.team1540.robot2024.Constants.Shooter.Pivot.HUB_SHOOT;

public class ShootSequence extends ParallelRaceGroup {

    public ShootSequence(Shooter shooter, Indexer indexer) {
        this(shooter, indexer, ()->HUB_SHOOT);
    }

    public ShootSequence(Shooter shooter, Indexer indexer, Supplier<ShooterSetpoint> setpoint) {
        addCommands(
                Commands.sequence(
//                        Commands.runOnce(shooter::zeroPivot),
                        new PrepareShooterCommand(shooter, setpoint.get())
                ),

                Commands.sequence(
//                        Commands.sequence(
//                                Commands.waitSeconds(1.5)
//                                Commands.waitUntil(() -> shooter.isPivotAtSetpoint() && shooter.areFlywheelsSpunUp())
                        // ) .withTimeout(1.5),
                        Commands.waitSeconds(1),
                        new IntakeAndFeed(indexer, () -> 1, () -> 0.5).withTimeout(0.5)
                )

                // TODO: Add a wait for having completed the shot (steady then current spike/velocity dip and then back down?)
        );
    }


    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
