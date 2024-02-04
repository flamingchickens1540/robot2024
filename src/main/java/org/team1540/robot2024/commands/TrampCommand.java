package org.team1540.robot2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.tramp.Tramp;

public class TrampCommand extends Command {

    private final Tramp tramp;

    public TrampCommand(Tramp tramp) {
        this.tramp = tramp;
        addRequirements(tramp);
    }

    @Override
    public void initialize() {
        tramp.setPercent(0.5);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return tramp.isBeamBreakBlocked();
    }

    @Override
    public void end(boolean interrupted) {
        tramp.stopTramp();
    }
}
