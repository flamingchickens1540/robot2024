package org.team1540.robot2024.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.tramp.Tramp;

public class IntakeAndFeed extends Command {
    private final Indexer indexer;
    private final double intakePercent;
    private final double feederPercent;

    public IntakeAndFeed(Indexer indexer, double intakePercent, double feederPercent) {
        this.indexer = indexer;
        this.intakePercent = intakePercent;
        this.feederPercent = feederPercent;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setIntakePercent(intakePercent);
        indexer.setFeederPercent(feederPercent);
    }
}
