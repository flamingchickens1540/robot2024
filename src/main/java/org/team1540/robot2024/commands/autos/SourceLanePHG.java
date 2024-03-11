package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.commands.shooter.AutoShootPrepare;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class SourceLanePHG extends AutoCommand {
    public SourceLanePHG(Drivetrain drivetrain, Shooter shooter, Indexer indexer){
        super("!SourceLanePHG");

        addPath(
                PathHelper.fromChoreoPath("SourceLanePHG.1", true, true),
                PathHelper.fromChoreoPath("SourceLanePHG.2")
        );

        addCommands(
                Commands.parallel(
                        new AutoShootPrepare(drivetrain, shooter),
                        Commands.sequence(
                                createSegmentSequence(drivetrain, indexer, 0),
                                createSegmentSequence(drivetrain, indexer, 1)
                        )
                )
        );
    }
}
