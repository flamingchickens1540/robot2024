package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2024.commands.drivetrain.DriveWithTargetingCommand;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.AutoShootPrepare;
import org.team1540.robot2024.commands.shooter.AutoShootPrepareWhileMoving;
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
                                                Commands.waitSeconds(0.1),
                                                Commands.waitUntil(()->!indexer.isNoteStaged()),
                                                Commands.waitSeconds(0.1)
                                        ).withTimeout(1.1),
                                        Commands.waitSeconds(0.2).andThen(IntakeAndFeed.withDefaults(indexer))
                                ),
                                createCancoderSegmentSequence(drivetrain, shooter, indexer, 0),
                                createSegmentSequence(drivetrain, shooter, indexer, 1),
                                Commands.runOnce(drivetrain::blockTags),
                                createSegmentSequence(drivetrain, shooter, indexer, 2),
                                createSegmentSequence(drivetrain, shooter, indexer, 3),
                                createSegmentSequence(drivetrain, shooter, indexer, 4)
                        )
                )
        );
    }
}
