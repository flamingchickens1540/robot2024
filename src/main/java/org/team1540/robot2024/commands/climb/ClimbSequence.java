package org.team1540.robot2024.commands.climb;


import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2024.Constants.Elevator.ElevatorState;
import org.team1540.robot2024.commands.elevator.ElevatorSetpointCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.tramp.Tramp;

public class ClimbSequence extends ParallelRaceGroup {


    public ClimbSequence(Drivetrain drivetrain, Elevator elevator, Tramp tramp, Indexer indexer, CommandXboxController controller) {
        addCommands(
                Commands.sequence(
                        new ClimbAlignment(drivetrain, elevator, tramp, indexer),
                        Commands.waitUntil(controller.a()),
                        new ElevatorSetpointCommand(elevator, ElevatorState.TOP),
                        Commands.waitSeconds(5), // Confirm that nothing will break. Also might need to be tuned if chain does weird things
                        Commands.startEnd(()->elevator.setPercent(-0.8), elevator::holdPosition, elevator).until(elevator::getLowerLimit)
                )
        );
    }
}
