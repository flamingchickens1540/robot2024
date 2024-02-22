package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class AmpLanePDEF extends AutoCommand {
    public AmpLanePDEF (Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("AmpLanePDEF");
        addPath(
                PathHelper.fromChoreoPath("AmpLanePDEF.1"),
                PathHelper.fromChoreoPath("AmpLanePDEF.2"),
                PathHelper.fromChoreoPath("AmpLanePDEF.3"),
                PathHelper.fromChoreoPath("AmpLanePDEF.4"),
                PathHelper.fromChoreoPath("AmpLanePDEF.5"),
                PathHelper.fromChoreoPath("AmpLanePDEF.6"),
                PathHelper.fromChoreoPath("AmpLanePDEF.7")
        );
        addCommands(
                getPath(0).getCommand(drivetrain),
                new ShootSequence(shooter, indexer),
                getPath(1).getCommand(drivetrain),
                new IntakeCommand(indexer, () -> false, 100),//TODO: tune this
                getPath(2).getCommand(drivetrain),
                new ShootSequence(shooter, indexer),
                getPath(3).getCommand(drivetrain),
                new IntakeCommand(indexer, () -> false, 100),
                getPath(4).getCommand(drivetrain),
                new ShootSequence(shooter, indexer),
                getPath(5).getCommand(drivetrain),
                new IntakeCommand(indexer, () -> false, 100),
                getPath(6).getCommand(drivetrain),
                new ShootSequence(shooter, indexer)
        );
    }
}
