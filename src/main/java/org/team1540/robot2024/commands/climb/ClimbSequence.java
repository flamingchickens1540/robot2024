package org.team1540.robot2024.commands.climb;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.Constants.Elevator.ElevatorState;
import org.team1540.robot2024.commands.ElevatorSetpointCommand;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.fakesubsystems.Hooks;

public class ClimbSequence extends SequentialCommandGroup {
    public ClimbSequence(Elevator elevator, Hooks hooks) { //TODO: Write servos no idea how they are supposed to work for now, add them somewhere :D
        addCommands(
                new ParallelCommandGroup(
                        new ElevatorSetpointCommand(elevator, ElevatorState.BOTTOM)
                        //TODO: Put whatever drive/alignment command we plan on using here
                ),
                new ElevatorSetpointCommand(elevator, ElevatorState.CLIMB),
                //TODO: Put whatever drive/alignment command we plan on using here
                new ElevatorSetpointCommand(elevator, ElevatorState.BOTTOM),
                hooks.deployHooksCommand() //TODO: Deploy hooks
        );
    }
}
