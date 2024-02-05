package org.team1540.robot2024.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.indexer.Indexer;

public class PrepareIndexerForShooter extends Command {

    private final Indexer indexer;

    public PrepareIndexerForShooter(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setFeederVelocity(1200);
    }

    @Override
    public boolean isFinished() {
        return indexer.isFeederAtSetpoint();
    }

}
