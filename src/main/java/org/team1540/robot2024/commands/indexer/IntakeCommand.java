package org.team1540.robot2024.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.indexer.Indexer;

import java.util.function.BooleanSupplier;

public class IntakeCommand extends Command {
    private final Indexer indexer;
    private final double percent;
    private final boolean shouldUseBeambreak;
    private final BooleanSupplier noteInTramp;


    public IntakeCommand(Indexer indexer, BooleanSupplier noteInTramp, double percent) {
        this(indexer, noteInTramp, percent, true);
    }

    public IntakeCommand(Indexer indexer, BooleanSupplier noteInTramp, double percent, boolean shouldUseBeambreak) {
        this.indexer = indexer;
        this.noteInTramp = noteInTramp;
        this.percent = percent;
        this.shouldUseBeambreak = shouldUseBeambreak;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setIntakePercent(percent);
    }

    @Override
    public boolean isFinished() {
        return shouldUseBeambreak && (indexer.isNoteStagedBack());
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopIntake();
    }
}
