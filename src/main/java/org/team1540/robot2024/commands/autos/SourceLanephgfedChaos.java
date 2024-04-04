package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class SourceLanephgfedChaos extends AutoCommand {
    public  SourceLanephgfedChaos (Shooter shooter, Indexer indexer, Drivetrain drivetrain) {
        super("SourceLanephgfedChaos");
        addPath(
                PathHelper.fromChoreoPath("SourceLanehgfedChaos.1", true, true),
                PathHelper.fromChoreoPath("SourceLanehgfedChaos.2"),
                PathHelper.fromChoreoPath("SourceLanehgfedChaos.3"),
                PathHelper.fromChoreoPath("SourceLanehgfedChaos.4"),
                PathHelper.fromChoreoPath("SourceLanehgfedChaos.5")
        );
        addCommands(
                new ShootSequence(shooter, indexer)
        );
        for (int i = 0; i < 5; ++i) {
            addCommands(
                    getPath(i).getCommand(drivetrain),
                    new IntakeAndFeed(indexer, () -> 1, () -> 0.05)//TODO: tune shoot speed to not eject note out of field
            );
        }
    }
}
