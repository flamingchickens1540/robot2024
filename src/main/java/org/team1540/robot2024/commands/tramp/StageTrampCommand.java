package org.team1540.robot2024.commands.tramp;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2024.commands.indexer.PrepareIndexerForTramp;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.tramp.Tramp;

public class StageTrampCommand extends SequentialCommandGroup {
    public StageTrampCommand(Tramp tramp, Indexer indexer) {
        addCommands(
                new PrepareIndexerForTramp(indexer),
                Commands.parallel(
                        Commands.runOnce(() -> indexer.setIntakePercent(0.5), indexer),
                        Commands.runOnce(() -> tramp.setPercent(0.5), tramp),
                        Commands.waitUntil(tramp::isNoteStaged)
                ),
                Commands.parallel(
                        Commands.runOnce(indexer::stopAll),
                        Commands.runOnce(tramp::stop, tramp)
                )

        );
    }
}
