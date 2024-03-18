package org.team1540.robot2024.commands.tramp;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.commands.elevator.ElevatorSetpointCommand;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.tramp.Tramp;

public class BackOutTramp extends SequentialCommandGroup {

    public BackOutTramp(Tramp tramp, Indexer indexer, Elevator elevator) {
        addCommands(
                new ElevatorSetpointCommand(elevator, Constants.Elevator.ElevatorState.BOTTOM),
                Commands.deadline(
                        Commands.sequence(
                                new WaitUntilCommand(indexer::isNoteStaged),
                                new WaitUntilCommand(() -> !indexer.isNoteStaged())
                        ),
                        new IntakeAndFeed(indexer, () -> -1, () -> 1),
                        Commands.runOnce(() -> tramp.setPercent(-1))
                ),
                new IntakeCommand(indexer, () -> false, 1)
        );
    }
}
