package org.team1540.robot2024.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.tramp.Tramp;

public class StageTrampCommand extends Command {

    private final Tramp tramp;
    private final Indexer indexer;

    public StageTrampCommand(Tramp tramp, Indexer indexer) {
        this.tramp = tramp;
        this.indexer = indexer;
        addRequirements(tramp, indexer);
    }
    @Override
    public void initialize() {
        tramp.setPercent(1); //TODO: Tune this
        indexer.setFeederPercent(-1);
        indexer.setIntakePercent(1);
    }

    @Override
    public boolean isFinished() {
        return tramp.isNoteStaged();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopFeeder();
        indexer.stopIntake();
        tramp.stop();
    }


}
