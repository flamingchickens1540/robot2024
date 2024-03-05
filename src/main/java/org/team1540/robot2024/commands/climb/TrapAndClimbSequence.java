package org.team1540.robot2024.commands.climb;

import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.Constants.Elevator.ElevatorState;
import org.team1540.robot2024.commands.elevator.ElevatorSetpointCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.fakesubsystems.Hooks;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.subsystems.tramp.Tramp;

import javax.imageio.metadata.IIOMetadataNode;

public class TrapAndClimbSequence extends SequentialCommandGroup {

    public TrapAndClimbSequence(Drivetrain drivetrain, Elevator elevator, Hooks hooks, Tramp tramp, Indexer indexer, Shooter shooter) {
        addCommands(
                new ClimbSequence(drivetrain, elevator, hooks, tramp, indexer, shooter),
                Commands.waitSeconds(5), //Confirm that nothing will break
                new ElevatorSetpointCommand(elevator, ElevatorState.TOP),
                Commands.runOnce(()->tramp.setDistanceToGo(3))
        );
    }
}
