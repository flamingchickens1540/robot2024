package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.shooter.ShooterSetpoint;
import org.team1540.robot2024.util.vision.AprilTagsCrescendo;

import java.util.function.Supplier;

import static org.team1540.robot2024.Constants.Shooter.Pivot.HUB_SHOOT;

public class ShootSequence extends ParallelRaceGroup {

    public ShootSequence(Supplier<Pose2d> positionSupplier, Shooter shooter, Indexer indexer) {
        this(shooter, indexer, () -> new ShooterSetpoint(
                Rotation2d.fromRadians(
                        Math.atan2(Constants.Targeting.SPEAKER_CENTER_HEIGHT - Constants.Shooter.Pivot.PIVOT_HEIGHT, positionSupplier.get().getTranslation().getDistance(
                                AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.SPEAKER_CENTER).toPose2d().getTranslation()
                        ))).minus(Constants.Shooter.Pivot.REAL_ZEROED_ANGLE),
                8000, 6000)
        );
    }

    public ShootSequence(Shooter shooter, Indexer indexer) {
        this(shooter, indexer, () -> HUB_SHOOT);
    }

    public ShootSequence(Shooter shooter, Indexer indexer, Supplier<ShooterSetpoint> setpoint) {
        addCommands(
                Commands.sequence(
//                        Commands.runOnce(shooter::zeroPivot),

                        new PrepareShooterCommand(shooter, setpoint)
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
