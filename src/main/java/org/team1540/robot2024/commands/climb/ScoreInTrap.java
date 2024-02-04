package org.team1540.robot2024.commands.climb;

import org.team1540.robot2024.subsystems.tramp.Tramp;

import edu.wpi.first.wpilibj2.command.Command;

//TODO: Write this command Tramp people :D
public class ScoreInTrap extends Command {
    private final Tramp tramp;
    public ScoreInTrap(Tramp tramp) {
        this.tramp = tramp;
        addRequirements(tramp);
    }
    @Override
    public void initialize() {
        //TODO: Score in trap :D
    }

    @Override
    public void end(boolean interrupted) {

    }
}
