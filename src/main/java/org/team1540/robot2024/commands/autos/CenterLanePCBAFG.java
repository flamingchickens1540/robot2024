package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.shooter.LeadingShootPrepare;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLanePCBAFG extends AutoCommand {

    public CenterLanePCBAFG(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("CenterLanePCBAFG");
        addPath(
                PathHelper.fromChoreoPath("CenterLanePCBAFG.1", false, true),
                PathHelper.fromChoreoPath("CenterLanePCBAFG.2"),
                PathHelper.fromChoreoPath("CenterLanePCBAFG.3"),
                PathHelper.fromChoreoPath("CenterLanePCBAFG.4"),
                PathHelper.fromChoreoPath("CenterLanePCBAFG.5")

        );

        addCommands(
                Commands.parallel(
                        new LeadingShootPrepare(drivetrain, shooter),
                        Commands.sequence(
                                Commands.deadline(
                                        Commands.sequence(
                                                Commands.waitSeconds(0.1),
                                                Commands.waitUntil(()->!indexer.isNoteStaged()),
                                                Commands.waitSeconds(0.1)
                                        ).withTimeout(1.1),
                                        Commands.waitSeconds(0.2).andThen(IntakeAndFeed.withDefaults(indexer))
                                ),
                                createSegmentSequence(drivetrain, shooter, indexer, 0, true, true, true, 0, 0.1),
                                createSegmentSequence(drivetrain, shooter, indexer, 1, false, true, true, 0, 0.1),
                                createSegmentSequence(drivetrain, shooter, indexer, 2, false, true, true, 0, 0.1),
                                createSegmentSequence(drivetrain, shooter, indexer, 3, false, true, true, 0, 0.1),
                                createSegmentSequence(drivetrain, shooter, indexer, 4, false, true, true, 0, 0.1)
                        )
                )
        );

        addRequirements(drivetrain, shooter, indexer);
    }
}
