package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLaneChaos extends AutoCommand {
    public CenterLaneChaos (Drivetrain drivetrain, Indexer indexer, Shooter shooter) {
        super("CenterLaneChaos");
        addPath(
                PathHelper.fromChoreoPath("CenterLaneChaos", true, true),
                PathHelper.fromChoreoPath("CenterLaneChaos.1"),
                PathHelper.fromChoreoPath("CenterLaneChaos.2")
        );
        addCommands(
                getPath(0).getCommand(drivetrain),
                new IntakeCommand(indexer, () -> false, 1),
                getPath(1).getCommand(drivetrain),
                new ShootSequence(shooter, indexer)
        );
    }
}
