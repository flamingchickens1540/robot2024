package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class SourceLanePHGSprint extends AutoCommand {
    public SourceLanePHGSprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer){
        super("SourceLanePHGSprint");

        addPath(
                PathHelper.fromChoreoPath("SourceLanePHGSprint.1", true, true),
                PathHelper.fromChoreoPath("SourceLanePHGSprint.2"),
                PathHelper.fromChoreoPath("SourceLanePHGSprint.3"),
                PathHelper.fromChoreoPath("SourceLanePHGSprint.4")
        );

        addCommands(
                getPath(0).getCommand(drivetrain),
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(3),
                        getPath(1).getCommand(drivetrain)
                ),
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(3),
                        getPath(2).getCommand(drivetrain)
                ),
                new ShootSequence(shooter, indexer),
                getPath(3).getCommand(drivetrain)
        );
    }
}
