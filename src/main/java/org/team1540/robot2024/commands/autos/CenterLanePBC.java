package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLanePBC extends AutoCommand {
    public CenterLanePBC (Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("CenterLanePBC");
        addPath(
                PathHelper.fromChoreoPath("CenterLanePBC.1"),
                PathHelper.fromChoreoPath("CenterLanePBC.2"),
                PathHelper.fromChoreoPath("CenterLanePBC.3")
        );
        addCommands(
                getPath(0).getCommand(drivetrain, true),
                new ShootSequence(shooter, indexer),
                new IntakeCommand(indexer, () -> false, 100), //TODO: tune this
                new ShootSequence(shooter, indexer),
                getPath(1).getCommand(drivetrain),
                getPath(2).getCommand(drivetrain),
                new IntakeCommand(indexer, () -> false, 100),
                getPath(3).getCommand(drivetrain),
                new ShootSequence(shooter, indexer)
        );
    }
}
