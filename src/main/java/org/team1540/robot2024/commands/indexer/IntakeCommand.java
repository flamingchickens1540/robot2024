package org.team1540.robot2024.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.tramp.Tramp;

public class IntakeCommand extends Command {
    private final Indexer indexer;
    private final Tramp tramp;

    public IntakeCommand(Indexer indexer, Tramp tramp) {
        this.indexer = indexer;
        this.tramp = tramp;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        if (indexer.isNoteStaged() || tramp.isNoteStaged()) indexer.stopIntake();
        else indexer.setIntakePercent(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopIntake();
    }
}
