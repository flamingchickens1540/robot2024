package org.team1540.robot2024.commands.climb;


import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2024.Constants.Elevator.ElevatorState;
import org.team1540.robot2024.commands.elevator.ElevatorSetpointCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.fakesubsystems.Hooks;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.subsystems.tramp.Tramp;

public class ClimbSequence extends ParallelRaceGroup {


    public ClimbSequence(Drivetrain drivetrain, Elevator elevator, Hooks hooks, Tramp tramp, Indexer indexer, Shooter shooter, CommandXboxController controller) {
        addCommands(
                Commands.startEnd(() -> {
                    shooter.stopPivot();
                    shooter.setPivotBrakeMode(false);
                }, () -> shooter.setPivotBrakeMode(true), shooter),
                Commands.sequence(
                        new ClimbAlignment(drivetrain, elevator, hooks, tramp, indexer, shooter),
                        Commands.waitUntil(controller.a()),
                        new ElevatorSetpointCommand(elevator, ElevatorState.TOP),
                        Commands.waitSeconds(5), //Confirm that nothing will break. Also might need to be tuned if chain does weird things
                        Commands.startEnd(()->elevator.setVoltage(-10), elevator::holdPosition, elevator).until(elevator::getLowerLimit)
                )
        );
    }
}
