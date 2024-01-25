package org.team1540.robot2024.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.Constants.Elevator.ElevatorState;

public class ClimbSequence extends SequentialCommandGroup {
    public ClimbSequence() { //TODO: Write servos no idea how they are supposed to work for now, add them somewhere :D
        addCommands(
                new ParallelCommandGroup(
                        new ElevatorSetpointCommand(ElevatorState.BOTTOM)
                        //TODO: Put whatever drive/alignment command we plan on using here
                ),
                new ElevatorSetpointCommand(ElevatorState.CLIMB),
                //TODO: Put whatever drive/alignment command we plan on using here
                new ElevatorSetpointCommand(ElevatorState.BOTTOM),
                new ActuateHooks(true) //TODO: Actuate hooks
        );
    }
}
