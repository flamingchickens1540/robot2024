package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLaneP extends AutoCommand {
    public CenterLaneP (Shooter shooter, Indexer indexer, Drivetrain drivetrain) {
        super("CenterLaneP");
        addPath(
                PathHelper.fromChoreoPath("CenterLaneP.1")
        );
        addCommands(
                getPath(0).getCommand(drivetrain, true),
                new ShootSequence(shooter, indexer)
        );
    }
}
