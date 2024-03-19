package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.shooter.AutoShootPrepare;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class AmpLanePABCSprint extends AutoCommand {
    public AmpLanePABCSprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer){
        super("!AmpLanePABCSprint");

        addPath(
                PathHelper.fromChoreoPath("AmpLanePABCSprint.1", true, true),
                PathHelper.fromChoreoPath("AmpLanePABCSprint.2"),
                PathHelper.fromChoreoPath("AmpLanePABCSprint.3"),
                PathHelper.fromChoreoPath("AmpLanePABCSprint.4")
//                PathHelper.fromChoreoPath("AmpLanePABCSprint.4")
        );

        addCommands(
                Commands.parallel(
                        new AutoShootPrepare(drivetrain, shooter),
                        Commands.sequence(
                                createSegmentSequence(drivetrain, shooter, indexer, 0),
                                IntakeAndFeed.withDefaults(indexer).withTimeout(2),
                                createSegmentSequence(drivetrain, shooter, indexer, 1),
                                createSegmentSequence(drivetrain, shooter, indexer, 2)
                        )
                ),
                getPath(3).getCommand(drivetrain)
        );
    }
}
