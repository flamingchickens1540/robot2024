package org.team1540.robot2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.fakesubsystems.Hooks;

public class DeployHooks extends Command {

    private final Hooks hooks;
    public DeployHooks(Hooks hooks) {
        //TODO: Not David's job
        this.hooks = hooks;
        addRequirements(hooks);
    }
    @Override
    public void initialize() {
        hooks.deployHooks();
    }

    @Override
    public void end(boolean interrupted) {

    }
}
