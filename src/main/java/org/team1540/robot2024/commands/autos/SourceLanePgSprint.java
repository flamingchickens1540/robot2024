package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class SourceLanePgSprint extends AutoCommand {
    public SourceLanePgSprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer){
        super("SourceLanePgSprint");

        addPath(
                PathHelper.fromChoreoPath("SourceLandPgSprint.1"),
                PathHelper.fromChoreoPath("SourceLandPgSprint.2")
        );

        addCommands(
                getPath(0).getCommand(drivetrain),
                new ShootSequence(shooter, indexer),
                getPath(1).getCommand(drivetrain)
        );
    }
}
