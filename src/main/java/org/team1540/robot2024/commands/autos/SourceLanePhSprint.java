package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class SourceLanePhSprint extends AutoCommand {
    public SourceLanePhSprint (Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("SourceLanePhSprint");
        addPath(
                PathHelper.fromChoreoPath("SourceLanePhSprint.1", true, true),
                PathHelper.fromChoreoPath("SourceLanePhSprint.2")
        );
        addCommands(
                new ShootSequence(shooter, indexer),
                getPath(0).getCommand(drivetrain, true),
                new IntakeCommand(indexer, () -> false, 1),
                getPath(1).getCommand(drivetrain)
        );
    }
}
