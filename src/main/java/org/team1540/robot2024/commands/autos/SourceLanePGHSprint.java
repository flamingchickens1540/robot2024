package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.commands.shooter.AutoShooterPrepare;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class SourceLanePGHSprint extends AutoCommand {
    public SourceLanePGHSprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer){
        super("!SourceLanePGHSprint");
        addPath(
                PathHelper.fromChoreoPath("SourceLanePGHSprint.1", true, true),
                PathHelper.fromChoreoPath("SourceLanePGHSprint.2"),
                PathHelper.fromChoreoPath("SourceLanePGHSprint.3"),
                PathHelper.fromChoreoPath("SourceLanePGHSprint.4")
        );

        addCommands(
                Commands.parallel(
                        new AutoShooterPrepare(drivetrain, shooter),
                        Commands.sequence(
                                createSegmentSequence(drivetrain, indexer, 0),
                                createSegmentSequence(drivetrain, indexer, 1),
                                createSegmentSequence(drivetrain, indexer, 2)
                        )
                ),
                getPath(3).getCommand(drivetrain)
        );
    }

}
