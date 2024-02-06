package org.team1540.robot2024.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.indexer.Indexer;

public class OuttakeCommand extends Command {

    private final Indexer indexer;

    public OuttakeCommand(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setIntakePercent(-0.5);
    }

    @Override
    public boolean isFinished() {
        return !indexer.isNoteStaged();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setIntakePercent(0);
    }

}
