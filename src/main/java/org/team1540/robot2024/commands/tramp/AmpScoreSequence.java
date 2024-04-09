package org.team1540.robot2024.commands.tramp;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.commands.elevator.ElevatorSetpointCommand;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.tramp.Tramp;

public class AmpScoreSequence extends SequentialCommandGroup {
    public AmpScoreSequence(Tramp tramp, Indexer indexer, Elevator elevator) {
        addCommands(
                new AmpScoreStageSequence(indexer, tramp, elevator).unless(tramp::isNoteStaged),
                new TrampOuttake(tramp),
                new ElevatorSetpointCommand(elevator, Constants.Elevator.ElevatorState.BOTTOM)
        );
    }
}
