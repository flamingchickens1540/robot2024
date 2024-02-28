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
                getPath(0).getCommand(drivetrain),
                new ShootSequence(shooter, indexer),
                new IntakeCommand(indexer, () -> false, 100), //TODO: Tune this
                new ShootSequence(shooter, indexer),
                getPath(1).getCommand(drivetrain),
                getPath(2).getCommand(drivetrain),
                new ShootSequence(shooter, indexer),
                new IntakeCommand(indexer, () -> false, 100),
                new ShootSequence(shooter, indexer)
        );
    }
}
