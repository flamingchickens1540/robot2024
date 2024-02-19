package org.team1540.robot2024.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.tramp.Tramp;

public class IntakeCommand extends Command {
    private final Indexer indexer;
    private final Tramp tramp;
    private final double percent;
    public IntakeCommand(Indexer indexer, Tramp tramp, double percent) {
        this.indexer = indexer;
        this.tramp = tramp;
        this.percent = percent;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setIntakePercent(percent);
    }

    @Override
    public boolean isFinished() {
        return tramp.isNoteStaged() || indexer.isNoteStaged();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopIntake();
    }
}
