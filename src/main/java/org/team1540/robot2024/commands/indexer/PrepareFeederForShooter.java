package org.team1540.robot2024.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.indexer.Indexer;

public class PrepareFeederForShooter extends Command {

    private final Indexer indexer;

    public PrepareFeederForShooter(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
//        indexer.setFeederVelocity(1200);
        indexer.setFeederPercent(0.5);
    }

    @Override
    public boolean isFinished() {
        return indexer.isFeederAtSetpoint();
    }

}
