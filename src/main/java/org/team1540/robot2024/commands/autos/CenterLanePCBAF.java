package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.AutoShooterPrepare;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLanePCBAF extends AutoCommand {

    public CenterLanePCBAF(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("!CenterLanePCBAF");
        addPath(
                PathHelper.fromChoreoPath("CenterLanePCBAF.1", true, true),
                PathHelper.fromChoreoPath("CenterLanePCBAF.2"),
                PathHelper.fromChoreoPath("CenterLanePCBAF.3"),
                PathHelper.fromChoreoPath("CenterLanePCBAF.4")
        );

        addCommands(
                new ShootSequence(shooter, indexer),
                Commands.parallel(
                        new AutoShooterPrepare(drivetrain, shooter),
                        Commands.sequence(
                                createSegmentSequence(drivetrain, indexer, 0),
                                createSegmentSequence(drivetrain, indexer, 1),
                                createSegmentSequence(drivetrain, indexer, 2),
                                createSegmentSequence(drivetrain, indexer, 3)
                        )
                )
        );

        addRequirements(drivetrain, shooter, indexer);
    }
}
