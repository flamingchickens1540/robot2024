package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.indexer.PrepareFeederForShooter;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.shooter.ShooterSetpoint;

import static org.team1540.robot2024.Constants.Shooter.Pivot.HUB_SHOOT;

public class ShootSequence extends SequentialCommandGroup {

    public ShootSequence(Shooter shooter, Indexer indexer) {
        this(shooter, indexer, HUB_SHOOT);
    }
    public ShootSequence(Shooter shooter, Indexer indexer, ShooterSetpoint setpoint) {
        addCommands(
                Commands.race(
                new PrepareShooterCommand(shooter, setpoint),
                        Commands.waitUntil(() -> shooter.isPivotAtSetpoint() && shooter.areFlywheelsSpunUp()).withTimeout(1.5)
                ),
//                Commands.sequence(
//                        new PrepareFeederForShooter(indexer).withTimeout(0.7), // TODO: 2/19/2024 fix this
                        new IntakeAndFeed(indexer, () -> 1, () -> 0.5)//.withTimeout(15)
//                )

                // TODO: Add a wait for having completed the shot (steady then current spike/velocity dip and then back down?)
        );
    }


    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
