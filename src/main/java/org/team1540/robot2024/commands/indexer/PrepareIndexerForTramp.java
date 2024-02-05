package org.team1540.robot2024.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.indexer.Indexer;

public class PrepareIndexerForTramp extends Command {

    private final Indexer indexer;

    // I'm not sure if the tramp should be done the same as the shooter here, but I think consistency wins?
    public PrepareIndexerForTramp(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setFeederVelocity(-600);
    }

    @Override
    public boolean isFinished() {
        return indexer.isFeederAtSetpoint();
    }

}
