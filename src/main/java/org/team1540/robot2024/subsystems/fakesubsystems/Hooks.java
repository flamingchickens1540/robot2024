package org.team1540.robot2024.subsystems.fakesubsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//TODO: Get rid of this class and make an actual one its just a temporary thing for David :D
public class Hooks extends SubsystemBase {

    public Hooks(){
        //something
    }

    //TODO: Whoever does this it should retract or deploy the hooks depending on if they are already deployed or retracted
    public void deployHooks() {

    }

    public void undeployHooks() {

    }

    /**
     * Factory method for deploying hooks
     */
    public Command deployHooksCommand() {
        return new InstantCommand(this::deployHooks, this);
    }

    /**
     * Factory method for un-deploying hooks
     */
    public Command undeployHooksCommand() {
        return new InstantCommand(this::undeployHooks, this);
    }
}
