package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.util.auto.PathHelper;

public class SourceLanehgfedChaos extends AutoCommand {
    public SourceLanehgfedChaos(Indexer indexer) {
        super("SourceLanephgfedChaos");
        addPath(
                PathHelper.fromChoreoPath("SourceLanehgfedChaos.1", true, true)
        );
        addCommands(
                new IntakeAndFeed(indexer, () -> 1, () -> 0.05)//TODO: tune shoot speed to not eject note out of field
        );
    }
}
