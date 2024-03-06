package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class AmpLaneChaos extends AutoCommand {
    public AmpLaneChaos (Drivetrain drivetrain, Indexer indexer, Shooter shooter) {
        super("AmpLaneChaos");
        addPath(
                PathHelper.fromChoreoPath("AmpLaneChaos.1", true, true),
                PathHelper.fromChoreoPath("AmpLaneChaos.2"),
                PathHelper.fromChoreoPath("AmpLaneChaos.3")
        );
        addCommands(
                getPath(0).getCommand(drivetrain),
                getPath(1).getCommand(drivetrain),
                new IntakeCommand(indexer, () -> false, 1),
                getPath(2).getCommand(drivetrain),
                new ShootSequence(shooter, indexer)
        );
    }
}
