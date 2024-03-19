package org.team1540.robot2024.commands.climb;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2024.Constants.Elevator.ElevatorState;
import org.team1540.robot2024.commands.elevator.ElevatorSetpointCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.subsystems.tramp.Tramp;


public class TrapAndClimbSequence extends SequentialCommandGroup {

    public TrapAndClimbSequence(Drivetrain drivetrain, Elevator elevator, Tramp tramp, Indexer indexer, Shooter shooter, CommandXboxController controller) {
        addCommands(
                new ClimbSequence(drivetrain, elevator, tramp, indexer, shooter, controller),//Confirm that nothing will break
                Commands.waitUntil(controller.a()),
                new ElevatorSetpointCommand(elevator, ElevatorState.TOP),
                Commands.runOnce(()->tramp.setDistanceToGo(3))
        );
    }
}
