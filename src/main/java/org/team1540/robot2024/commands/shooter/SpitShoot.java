package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.shooter.ShooterSetpoint;
import org.team1540.robot2024.util.vision.AprilTagsCrescendo;

import java.util.function.Supplier;

public class SpitShoot extends ParallelCommandGroup {
    public SpitShoot(Shooter shooter, Indexer indexer, double leftSetpoint, double rightSetpoint, double indexerPercent, double feederPercent, double waitSeconds) {
        addCommands(
                Commands.waitSeconds(waitSeconds).andThen(new IntakeAndFeed(indexer, ()->indexerPercent, ()->feederPercent)),
                new PrepareShooterCommand(shooter, ()-> new ShooterSetpoint(0, leftSetpoint, rightSetpoint))
        );
    }

    public SpitShoot(Shooter shooter, Indexer indexer){
        this(shooter, indexer, Constants.Shooter.Flywheels.LEFT_RPM, Constants.Shooter.Flywheels.RIGHT_RPM, 1, 1, 0);
    }
}
