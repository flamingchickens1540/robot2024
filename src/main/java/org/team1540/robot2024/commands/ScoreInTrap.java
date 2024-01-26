package org.team1540.robot2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.fakesubsystems.Tramp;

public class ScoreInTrap extends Command {
    private final Tramp tramp;
    public ScoreInTrap(Tramp tramp) {
        this.tramp = tramp;
        addRequirements(tramp);
    }
    @Override
    public void initialize() {
        tramp.scoreInTrap();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
