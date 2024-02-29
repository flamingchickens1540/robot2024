package org.team1540.robot2024.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.Constants.Elevator.ElevatorState;
import org.team1540.robot2024.commands.elevator.ElevatorSetpointCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.fakesubsystems.Hooks;
import org.team1540.robot2024.subsystems.tramp.Tramp;

import javax.imageio.metadata.IIOMetadataNode;

public class TrapAndClimbSequence extends SequentialCommandGroup {

    public TrapAndClimbSequence(Drivetrain drivetrain, Elevator elevator, Hooks hooks, Tramp tramp) {
        addCommands(
//                new ClimbSequence(drivetrain, elevator, hooks), //Climb
                new ElevatorSetpointCommand(elevator, ElevatorState.TOP),
                new ParallelDeadlineGroup(
                        new WaitCommand(Constants.Tramp.TRAP_SCORING_TIME_SECONDS),
                        new InstantCommand(()->tramp.setPercent(1))
//                        new ScoreInTrap(tramp) //TODO: Do whatever to this but not my job
                )
        );
    }
}
