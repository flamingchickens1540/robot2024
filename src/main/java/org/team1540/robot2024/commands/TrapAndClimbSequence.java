package org.team1540.robot2024.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2024.Constants.Elevator.ElevatorState;
import org.team1540.robot2024.subsystems.fakesubsystems.Elevator;
import org.team1540.robot2024.subsystems.fakesubsystems.Hooks;
import org.team1540.robot2024.subsystems.fakesubsystems.Tramp;

public class TrapAndClimbSequence extends SequentialCommandGroup {

    public TrapAndClimbSequence(Elevator elevator, Hooks hooks, Tramp tramp) {
        addCommands(
                new ClimbSequence(elevator, hooks), //Climb
                new WaitCommand(0.1), //TODO: Perhaps remove this or change it depending on how climbing turns out to be
                new ElevatorSetpointCommand(elevator, ElevatorState.TRAP),
                new ScoreInTrap(tramp), //TODO: Do whatever to this but not my job
                new ElevatorSetpointCommand(elevator, ElevatorState.BOTTOM)
        );
    }
}
