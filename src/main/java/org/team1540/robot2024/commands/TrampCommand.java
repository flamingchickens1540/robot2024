package org.team1540.robot2024.commands;

import org.team1540.robot2024.subsystems.tramp.Tramp;

import edu.wpi.first.wpilibj2.command.Command;

public class TrampCommand extends Command {

    private final Tramp tramp;

    public TrampCommand(Tramp tramp) {
        this.tramp = tramp;
        addRequirements(tramp);
    }

    public void getNote(Tramp tramp) {
        if (tramp.isBeamBreakBlocked()) {
            tramp.end();
        } else {
            tramp.scoreTrampCommand();
        }
    }

    public void shootTramp(Tramp tramp) {
        tramp.scoreTrampCommand();
    }

}
