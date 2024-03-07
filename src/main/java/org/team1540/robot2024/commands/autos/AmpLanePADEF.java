package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class AmpLanePADEF extends AutoCommand {
    public AmpLanePADEF(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("AmpLanePDEF");
        addPath(
                PathHelper.fromChoreoPath("AmpLanePDEF.1", true, true),
                PathHelper.fromChoreoPath("AmpLanePDEF.2"),
                PathHelper.fromChoreoPath("AmpLanePDEF.3"),
                PathHelper.fromChoreoPath("AmpLanePDEF.4"),
                PathHelper.fromChoreoPath("AmpLanePDEF.5"),
                PathHelper.fromChoreoPath("AmpLanePDEF.6"),
                PathHelper.fromChoreoPath("AmpLanePDEF.7")
        );
        addCommands(
                new ShootSequence(shooter, indexer),
                getPath(0).getCommand(drivetrain, true),
                new IntakeCommand(indexer, () -> false, 1),
                new ShootSequence(shooter, indexer),
                getPath(1).getCommand(drivetrain),
                new ParallelCommandGroup(
                new IntakeCommand(indexer, () -> false, 1),
                getPath(2).getCommand(drivetrain)
                ),
                new ShootSequence(shooter, indexer),
                getPath(3).getCommand(drivetrain),
                new ParallelCommandGroup(
                new IntakeCommand(indexer, () -> false, 1),
                getPath(2).getCommand(drivetrain)
                ),
                new ShootSequence(shooter, indexer),
                getPath(3).getCommand(drivetrain),
                new ParallelCommandGroup(
                new IntakeCommand(indexer, () -> false, 1),
                getPath(2).getCommand(drivetrain)
                ),
                new ShootSequence(shooter, indexer)
        );
    }
}
