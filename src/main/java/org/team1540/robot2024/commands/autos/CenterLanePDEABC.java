package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.shooter.LeadingShootPrepare;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLanePDEABC extends AutoCommand {

    public CenterLanePDEABC(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("!CenterLanePDEABC");

        addPath(
                PathHelper.fromChoreoPath("CenterLanePDEABC.1", true, true),
                PathHelper.fromChoreoPath("CenterLanePDEABC.2"),
                PathHelper.fromChoreoPath("CenterLanePDEABC.3"),
                PathHelper.fromChoreoPath("CenterLanePDEABC.4"),
                PathHelper.fromChoreoPath("CenterLanePDEABC.5")
        );

        addCommands(
                Commands.parallel(
                        new LeadingShootPrepare(drivetrain, shooter),
                        Commands.sequence(
                                Commands.deadline(
                                        Commands.sequence(
                                                Commands.waitUntil(()->!indexer.isNoteStagedBack()),
                                                Commands.waitSeconds(0.1)
                                        ).withTimeout(1.1),
                                        IntakeAndFeed.withDefaults(indexer)
                                ),
                                createSegmentSequence(drivetrain, shooter, indexer, 0, true, true, true, 0, 0.1),
                                Commands.runOnce(drivetrain::unblockTags),
                                createSegmentSequence(drivetrain, shooter, indexer, 1, false, true, true, 0, 0.1),
                                Commands.runOnce(drivetrain::blockTags),
                                createSegmentSequence(drivetrain, shooter, indexer, 2, false, true, true, 0, 0.1),
                                createSegmentSequence(drivetrain, shooter, indexer, 3, false, true, true, 0, 0.1),
                                Commands.parallel(
                                        getPath(4).getCommand(drivetrain),
                                        IntakeAndFeed.withDefaults(indexer)
                                )
//                                createSegmentSequence(drivetrain, shooter, indexer, 4, false, true, true, 0, 0.1)
                        )
                )
        );
    }
}
