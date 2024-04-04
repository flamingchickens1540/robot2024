package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
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
                PathHelper.fromChoreoPath("SourceLanephgfedChaos.1", true, true),
                PathHelper.fromChoreoPath("SourceLanephgfedChaos.2"),
                PathHelper.fromChoreoPath("SourceLanephgfedChaos.3"),
                PathHelper.fromChoreoPath("SourceLanephgfedChaos.4"),
                PathHelper.fromChoreoPath("SourceLanephgfedChaos.5")
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
