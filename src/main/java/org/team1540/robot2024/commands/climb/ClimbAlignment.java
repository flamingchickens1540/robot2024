package org.team1540.robot2024.commands.climb;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2024.Constants;
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

public class ClimbAlignment extends ParallelRaceGroup {

    private final Drivetrain drivetrain;

    public static final PathConstraints STAGE_PATH_CONSTRAINTS = new PathConstraints(
            1.0, 0.5,
            1,
            0.3);

    public ClimbAlignment(Drivetrain drivetrain, Elevator elevator, Hooks hooks, Tramp tramp, Indexer indexer, Shooter shooter){
        this.drivetrain = drivetrain;
        addCommands(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new ElevatorSetpointCommand(elevator, Constants.Elevator.ElevatorState.BOTTOM),
                                    new StageTrampCommand(tramp, indexer).onlyIf(indexer::isNoteStaged)
                            ),
                            new ProxyCommand(() -> climbPath(drivetrain::getPose, 1))
                    ),
                    Commands.runOnce(() -> drivetrain.setBrakeMode(false)),
                    Commands.waitSeconds(5), //Confirm that nothing will break
                    new ElevatorSetpointCommand(elevator, Constants.Elevator.ElevatorState.CLIMB),
                    Commands.runOnce(() -> drivetrain.setBrakeMode(true)),
                    Commands.waitSeconds(5), //Confirm that nothing will break
                    new ProxyCommand(() -> climbPath(drivetrain::getPose, 2))
                ),
                new StartEndCommand(()->{}, ()->{
                    drivetrain.setBrakeMode(true);
                    drivetrain.unblockTags();
                })
        );
    }

    private Command climbPath(Supplier<Pose2d> position, int index){
        double far = position.get().getTranslation().getDistance(
                AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.CLIMB_FAR).getTranslation().toTranslation2d());
        double amp = position.get().getTranslation().getDistance(
                AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.CLIMB_AMP).getTranslation().toTranslation2d());
        double source = position.get().getTranslation().getDistance(
                AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.CLIMB_SOURCE).getTranslation().toTranslation2d());
        Command pathCmd;
        if(far < amp && far < source){
            pathCmd= AutoBuilder.pathfindThenFollowPath(PathHelper.fromChoreoPath("Tag14" + index).getPath(), STAGE_PATH_CONSTRAINTS);
        }
        else if(amp < source){
            pathCmd= AutoBuilder.pathfindThenFollowPath(PathHelper.fromChoreoPath("Tag15" + index).getPath(), STAGE_PATH_CONSTRAINTS);
        }
        else{
            pathCmd= AutoBuilder.pathfindThenFollowPath(PathHelper.fromChoreoPath("Tag16" + index).getPath(), STAGE_PATH_CONSTRAINTS);
        }
        return Commands.runOnce(drivetrain::blockTags).andThen(pathCmd).andThen(Commands.runOnce(drivetrain::unblockTags));
    }
}
