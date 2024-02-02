package org.team1540.robot2024.commands;

import org.team1540.robot2024.subsystems.tramp.Tramp;

import edu.wpi.first.wpilibj2.command.Command;

public class TrampCommand extends Command {

    private final Tramp tramp;

    public TrampCommand(Tramp tramp) {
        this.tramp = tramp;
        addRequirements(tramp);
    }

    @Override
    public void initialize() {
        if (tramp.isBeamBreakBlocked()) {
            tramp.stopTramp();
        } else {
            tramp.scoreTrampCommand();
        }
    }

    @Override
    public void execute() {
        tramp.scoreTrampCommand();
    }

    @Override
    public void end(boolean interrupted) {
        tramp.stopTramp();
    }
}
