package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.util.auto.PathHelper;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.commands.shooter.LeadingShootPrepare;

public class SourceLaneCBASprint extends AutoCommand {
    public SourceLaneCBASprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("SourceLaneCBASprint");

        addPath(
                PathHelper.fromChoreoPath("SourceLaneCBASprint.1", true, true),
                PathHelper.fromChoreoPath("SourceLaneCBASprint.2"),
                PathHelper.fromChoreoPath("SourceLaneCBASprint.3"),
                PathHelper.fromChoreoPath("SourceLaneCBASprint.4"),
                PathHelper.fromChoreoPath("SourceLaneCBASprint.5")
        );
        addCommands(
                Commands.parallel(
                        new LeadingShootPrepare(drivetrain, shooter),
                        Commands.sequence(
                                createCancoderSegmentSequence(drivetrain, shooter, indexer, 0),
                                createSegmentSequence(drivetrain, shooter, indexer, 1),
                                createSegmentSequence(drivetrain, shooter, indexer, 2),
                                createSegmentSequence(drivetrain, shooter, indexer, 3),
                                getPath(4).getCommand(drivetrain)
                        )
                )
        );
    }
}