package org.team1540.robot2024.commands.tramp;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.commands.elevator.ElevatorSetpointCommand;
import org.team1540.robot2024.commands.indexer.StageTrampCommand;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.tramp.Tramp;

public class AmpScoreStageSequence extends SequentialCommandGroup {
    public AmpScoreStageSequence(Indexer indexer, Tramp tramp, Elevator elevator){
        addCommands(
                new ElevatorSetpointCommand(elevator, Constants.Elevator.ElevatorState.BOTTOM).withTimeout(3),
                new StageTrampCommand(tramp, indexer).withTimeout(3),
                new ParallelCommandGroup(
                        Commands.runOnce(() -> tramp.setDistanceToGo(1)),
                        new ElevatorSetpointCommand(elevator, Constants.Elevator.ElevatorState.AMP)
                ).withTimeout(5)
        );
    }
}
