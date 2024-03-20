package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class SourceLanePHGFED extends AutoCommand {
    public SourceLanePHGFED (Drivetrain drivetrain, Indexer indexer, Shooter shooter) {
        super("SourceLanePHGFED");
        addPath(
                PathHelper.fromChoreoPath("SourceLanePHGFED.1", true, true),
                PathHelper.fromChoreoPath("SourceLanePHGFED.2"),
                PathHelper.fromChoreoPath("SourceLanePHGFED.3"),
                PathHelper.fromChoreoPath("SourceLanePHGFED.4"),
                PathHelper.fromChoreoPath("SourceLanePHGFED.5"),
                PathHelper.fromChoreoPath("SourceLanePHGFED.6"),
                PathHelper.fromChoreoPath("SourceLanePHGFED.7"),
                PathHelper.fromChoreoPath("SourceLanePHGFED.8"),
                PathHelper.fromChoreoPath("SourceLanePHGFED.9"),
                PathHelper.fromChoreoPath("SourceLanePHGFED.10")
        );
        addCommands(
                new ShootSequence(shooter, indexer),
                getPath(0).getCommand(drivetrain),
                new ParallelCommandGroup(
                new IntakeCommand(indexer, () -> false, 1),
                getPath(1).getCommand(drivetrain)
                ),
                new ShootSequence(shooter, indexer),
                getPath(0).getCommand(drivetrain),
                new ParallelCommandGroup(
                new IntakeCommand(indexer, () -> false, 1),
                getPath(1).getCommand(drivetrain)
                ),
                new ShootSequence(shooter, indexer),
                getPath(0).getCommand(drivetrain),
                new ParallelCommandGroup(
                new IntakeCommand(indexer, () -> false, 1),
                getPath(1).getCommand(drivetrain)
                ),
                new ShootSequence(shooter, indexer),
                getPath(0).getCommand(drivetrain),
                new ParallelCommandGroup(
                new IntakeCommand(indexer, () -> false, 1),
                getPath(1).getCommand(drivetrain)
                ),
                new ShootSequence(shooter, indexer),
                getPath(0).getCommand(drivetrain),
                new ParallelCommandGroup(
                new IntakeCommand(indexer, () -> false, 1),
                getPath(1).getCommand(drivetrain)
                ),
                new ShootSequence(shooter, indexer)
        );
    }
}
