package org.team1540.robot2024.commands.tramp;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.commands.elevator.ElevatorSetpointCommand;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.tramp.Tramp;

public class TrampScoreSequence extends SequentialCommandGroup {
    public TrampScoreSequence(Tramp tramp, Indexer indexer, Elevator elevator) {
        addCommands(
                new TrampStageSequence(indexer, tramp, elevator).unless(tramp::isNoteStaged),
                new TrampShoot(tramp),
                new ElevatorSetpointCommand(elevator, Constants.Elevator.ElevatorState.BOTTOM)
        );
    }
}
