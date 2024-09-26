package org.team1540.robot2024.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.indexer.Indexer;

public class ContinuousIntakeCommand extends Command {
    private final Indexer indexer;
    private final double percent;



    public ContinuousIntakeCommand(Indexer indexer, double percent) {
        this.indexer = indexer;
        this.percent = percent;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        if (indexer.getNoteState() == Indexer.NotePosition.INDEXER) {
            indexer.stopIntake();
        } else {
            indexer.setIntakePercent(percent);
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopIntake();
    }
}
