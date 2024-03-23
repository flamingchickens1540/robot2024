package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.shooter.ShooterSetpoint;
import org.team1540.robot2024.util.vision.AprilTagsCrescendo;

import java.util.function.Supplier;

import static org.team1540.robot2024.Constants.Shooter.Pivot.HUB_SHOOT;

public class ShootSequence extends ParallelRaceGroup {

    public ShootSequence(Shooter shooter, Indexer indexer) {
        this(shooter, indexer, () -> HUB_SHOOT);
    }
    public ShootSequence(Shooter shooter, Indexer indexer, Supplier<ShooterSetpoint> setpoint) {
        this(shooter, indexer, setpoint, 1);
    }
    public ShootSequence(Shooter shooter, Indexer indexer, Supplier<ShooterSetpoint> setpoint, double waitTime) {
        addCommands(
                new PrepareShooterCommand(shooter, setpoint),
                Commands.sequence(
                        Commands.waitSeconds(waitTime),
                        IntakeAndFeed.withDefaults(indexer).withTimeout(0.5)
                )

                // TODO: Add a wait for having completed the shot (steady then current spike/velocity dip and then back down?)
        );
    }

    public static Command forAutoSubwoofer(Shooter shooter, Indexer indexer) {
        return Commands.race(
                new PrepareShooterCommand(shooter, ()-> shooter.lerp.get(Units.feetToMeters(3))),
                Commands.sequence(
                        Commands.waitSeconds(1),
                        Commands.runOnce(shooter::zeroPivotToCancoder),
                        IntakeAndFeed.withDefaults(indexer).withTimeout(0.5)
                )

        );
    }


    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
