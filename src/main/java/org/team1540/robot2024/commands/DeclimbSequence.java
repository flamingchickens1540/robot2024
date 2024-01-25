package org.team1540.robot2024.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.Constants.Elevator.ElevatorState;

public class DeclimbSequence extends SequentialCommandGroup {
    public DeclimbSequence() {
        addCommands(
            new ElevatorSetpointCommand(ElevatorState.BOTTOM),
            new ActuateHooks(false), //Release hooks
            new ElevatorSetpointCommand(ElevatorState.TOP)
        );
    }
}
