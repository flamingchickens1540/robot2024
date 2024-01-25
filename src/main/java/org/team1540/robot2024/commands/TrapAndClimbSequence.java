package org.team1540.robot2024.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2024.Constants.Elevator.ElevatorState;

public class TrapAndClimbSequence extends SequentialCommandGroup {
    public TrapAndClimbSequence() {
        addCommands(
                new ClimbSequence(), //Climb
                new WaitCommand(0.1), //TODO: Perhaps remove this or change it depending on how climbing turns out to be
                new ElevatorSetpointCommand(ElevatorState.TRAP),
                new ScoreInTrap(), //TODO: Do whatever to this but not my job
                new ElevatorSetpointCommand(ElevatorState.BOTTOM)
        );
    }
}
