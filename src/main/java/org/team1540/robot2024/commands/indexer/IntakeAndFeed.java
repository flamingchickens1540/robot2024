package org.team1540.robot2024.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.indexer.Indexer;

import java.util.function.DoubleSupplier;

public class IntakeAndFeed extends Command {
    private final Indexer indexer;
    private final DoubleSupplier intakePercent;
    private final DoubleSupplier feederPercent;

    public static Command withDefaults(Indexer indexer) {
        return new IntakeAndFeed(indexer, () -> 0.9, () -> 1);
    }

    public IntakeAndFeed(Indexer indexer, DoubleSupplier intakePercent, DoubleSupplier feederPercent) {
        this.indexer = indexer;
        this.intakePercent = intakePercent;
        this.feederPercent = feederPercent;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setIntakePercent(intakePercent.getAsDouble());
        indexer.setFeederPercent(feederPercent.getAsDouble());
    }

    @Override
    public void end(boolean isInterrupted) {
        indexer.stopAll();
    }
}
