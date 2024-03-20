package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class AmpLanePDEFGH extends AutoCommand {
    public AmpLanePDEFGH (Drivetrain drivetrain, Indexer indexer, Shooter shooter) {
        super("AmpLanePDEFGH");
        addPath(
                PathHelper.fromChoreoPath("AmpLanePDEFGH.1", true, true),
                PathHelper.fromChoreoPath("AmpLanePDEFGH.2"),
                PathHelper.fromChoreoPath("AmpLanePDEFGH.3"),
                PathHelper.fromChoreoPath("AmpLanePDEFGH.4"),
                PathHelper.fromChoreoPath("AmpLanePDEFGH.5"),
                PathHelper.fromChoreoPath("AmpLanePDEFGH.6"),
                PathHelper.fromChoreoPath("AmpLanePDEFGH.7"),
                PathHelper.fromChoreoPath("AmpLanePDEFGH.8"),
                PathHelper.fromChoreoPath("AmpLanePDEFGH.9"),
                PathHelper.fromChoreoPath("AmpLanePDEFGH.10")
        );
        addCommands(
                new ShootSequence(shooter, indexer),
                getPath(0).getCommand(drivetrain),
                new ParallelCommandGroup(
                new IntakeCommand(indexer, () -> false, 1),
                getPath(1).getCommand(drivetrain)
                ),
                new ShootSequence(shooter, indexer),
                getPath(2).getCommand(drivetrain),
                new ParallelCommandGroup(
                new IntakeCommand(indexer, () -> false, 1),
                getPath(3).getCommand(drivetrain)
                ),
                new ShootSequence(shooter, indexer),
                getPath(4).getCommand(drivetrain),
                new ParallelCommandGroup(
                new IntakeCommand(indexer, () -> false, 1),
                getPath(5).getCommand(drivetrain)
                ),
                new ShootSequence(shooter, indexer),
                getPath(6).getCommand(drivetrain),
                new ParallelCommandGroup(
                new IntakeCommand(indexer, () -> false, 1),
                getPath(7).getCommand(drivetrain)
                ),
                new ShootSequence(shooter, indexer),
                getPath(8).getCommand(drivetrain),
                new ParallelCommandGroup(
                new IntakeCommand(indexer, () -> false, 1),
                getPath(9).getCommand(drivetrain)
                ),
                new ShootSequence(shooter, indexer)
        );
    }
}
