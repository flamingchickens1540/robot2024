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

    // It should not be necessary to deal with the case of the note in the robot and not being detected,
    // because the indexer will always be being used when the note is in transition (?)
    // but maybe just for code safety this is good
    // also it is hard to imagine that being a problem
    // @zach
    @Override
    public void execute() {
        if (indexer.isNoteStaged() || tramp.isNoteStaged()) {
            indexer.stopIntake();
        } else {
            indexer.setIntakePercent(0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopIntake();
    }
}
