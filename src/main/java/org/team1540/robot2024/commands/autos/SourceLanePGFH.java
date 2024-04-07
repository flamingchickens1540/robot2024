package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.commands.shooter.AutoShootPrepare;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class SourceLanePGFH extends AutoCommand {
    private static final double SHOT_WAIT = 0.2;
    public SourceLanePGFH(Drivetrain drivetrain, Shooter shooter, Indexer indexer){
        super("!SourceLanePGFH");
        addPath(
                PathHelper.fromChoreoPath("SourceLanePGFH.1", true, true),
                PathHelper.fromChoreoPath("SourceLanePGFH.2"),
                PathHelper.fromChoreoPath("SourceLanePGFH.3"),
                PathHelper.fromChoreoPath("SourceLanePGFH.4")
        );

        addCommands(
                Commands.parallel(
                        new AutoShootPrepare(drivetrain, shooter),
                        Commands.sequence(
                                createSegmentSequence(drivetrain, shooter, indexer, 0, true, true, true, 0, 0, true),
                                createSegmentSequence(drivetrain, shooter, indexer, 1, false, true, true, 0, 0, true),
                                createSegmentSequence(drivetrain, shooter, indexer, 2, false, true, true, 0,0, true),
                                createSegmentSequence(drivetrain, shooter, indexer, 3, false, true, true, 0, 0, false)
                        )
                )
        );
    }

}
