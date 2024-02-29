package org.team1540.robot2024.commands.tramp;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.commands.elevator.ElevatorSetpointCommand;
import org.team1540.robot2024.commands.indexer.StageTrampCommand;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.tramp.Tramp;

public class TrampStageSequence extends SequentialCommandGroup {
    public TrampStageSequence(Indexer indexer, Tramp tramp, Elevator elevator){
        addCommands(
                new ElevatorSetpointCommand(elevator, Constants.Elevator.ElevatorState.BOTTOM),
                new StageTrampCommand(tramp, indexer),
                Commands.runOnce(() -> tramp.setDistanceToGo(1)),
                new ElevatorSetpointCommand(elevator, Constants.Elevator.ElevatorState.AMP)
        );
    }
}
