package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class AmpLanePAB extends AutoCommand {
    public AmpLanePAB (Shooter shooter, Indexer indexer, Drivetrain drivetrain) {
        super("AmpLanePAB");
        addPath(
                PathHelper.fromChoreoPath("AmpLanePAB.1", true, true),
                PathHelper.fromChoreoPath("AmpLanePAB.2"),
                PathHelper.fromChoreoPath("AmpLanePAB.3")
        );
        addCommands(
                new ShootSequence(shooter, indexer),
                getPath(0).getCommand(drivetrain, true),
                new IntakeCommand(indexer, () -> false, 1),
                new ShootSequence(shooter, indexer),
                getPath(1).getCommand(drivetrain),
                new IntakeCommand(indexer, () -> false, 1),
                getPath(2).getCommand(drivetrain),
                new ShootSequence(shooter, indexer)
        );
    }
}
