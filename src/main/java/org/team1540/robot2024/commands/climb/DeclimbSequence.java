package org.team1540.robot2024.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.Constants.Elevator.ElevatorState;
import org.team1540.robot2024.commands.ElevatorSetpointCommand;
import org.team1540.robot2024.subsystems.fakesubsystems.Elevator;
import org.team1540.robot2024.subsystems.fakesubsystems.Hooks;

public class DeclimbSequence extends SequentialCommandGroup {
    public DeclimbSequence(Elevator elevator, Hooks hooks) {
        addCommands(
            new ElevatorSetpointCommand(elevator, ElevatorState.BOTTOM),
            hooks.undeployHooksCommand(), //Release hooks
            new ElevatorSetpointCommand(elevator, ElevatorState.TOP)
        );
    }
}
