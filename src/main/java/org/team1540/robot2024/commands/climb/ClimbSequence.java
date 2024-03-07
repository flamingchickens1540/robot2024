package org.team1540.robot2024.commands.climb;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.Constants.Elevator.ElevatorState;
import org.team1540.robot2024.commands.elevator.ElevatorSetpointCommand;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.fakesubsystems.Hooks;
import org.team1540.robot2024.util.auto.PathHelper;

public class ClimbSequence extends SequentialCommandGroup {
    public ClimbSequence(Elevator elevator, Hooks hooks) { //TODO: Write servos no idea how they are supposed to work for now, add them somewhere :D
        addCommands(
                new ParallelCommandGroup(
                        new ElevatorSetpointCommand(elevator, ElevatorState.BOTTOM),
                        AutoBuilder.pathfindThenFollowPath(PathHelper.fromChoreoPath("Tag14").getPath(), Constants.Auto.PATH_CONSTRAINTS)
                        //TODO: Put whatever drive/alignment command we plan on using here
                ),
                Commands.waitSeconds(1),
                new ElevatorSetpointCommand(elevator, ElevatorState.CLIMB),
                Commands.waitSeconds(1),
                //TODO: Put whatever drive/alignment command we plan on using here
                new ElevatorSetpointCommand(elevator, ElevatorState.BOTTOM)
//                hooks.deployHooksCommand() //TODO: Deploy hooks
        );
    }
}
