package org.team1540.robot2024.commands.climb;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.Constants.Elevator.ElevatorState;
import org.team1540.robot2024.commands.elevator.ElevatorSetpointCommand;
import org.team1540.robot2024.commands.indexer.StageTrampCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.fakesubsystems.Hooks;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.subsystems.tramp.Tramp;
import org.team1540.robot2024.util.auto.PathHelper;
import org.team1540.robot2024.util.vision.AprilTagsCrescendo;

import java.util.function.Supplier;

public class ClimbSequence extends ParallelRaceGroup {

    public static final PathConstraints STAGE_PATH_CONSTRAINTS = new PathConstraints(
            1.0, 0.5,
            1,
            0.3);

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
//                        new ElevatorSetpointCommand(elevator, ElevatorState.BOTTOM)
                        Commands.startEnd(()->elevator.setVoltage(-10), elevator::holdPosition, elevator).until(elevator::getLowerLimit)
                )
//                new ElevatorSetpointCommand(elevator, ElevatorState.CLIMB),
//                        Commands.runOnce(() -> drivetrain.setBrakeMode(true)),
                        //TODO: Put whatever drive/alignment command we plan on using here
//                new ElevatorSetpointCommand(elevator, ElevatorState.BOTTOM)
//                hooks.deployHooksCommand() //TODO: Deploy hooks
        );
    }
}
