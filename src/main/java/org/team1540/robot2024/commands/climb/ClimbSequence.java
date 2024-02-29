package org.team1540.robot2024.commands.climb;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.Constants.Elevator.ElevatorState;
import org.team1540.robot2024.commands.elevator.ElevatorSetpointCommand;
import org.team1540.robot2024.commands.indexer.StageTrampCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.fakesubsystems.Hooks;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.tramp.Tramp;
import org.team1540.robot2024.util.auto.PathHelper;
import org.team1540.robot2024.util.vision.AprilTagsCrescendo;

import java.util.function.Supplier;

public class ClimbSequence extends ParallelCommandGroup {
    private final Drivetrain drivetrain;

    public static final PathConstraints STAGE_PATH_CONSTRAINTS = new PathConstraints(
            1.0, 0.5,
            1,
            0.3);

    public ClimbSequence(Drivetrain drivetrain, Elevator elevator, Hooks hooks, Tramp tramp, Indexer indexer) {
        this.drivetrain = drivetrain;
//        PathHelper pathHelper = PathHelper.fromChoreoPath("Tag14");
//        PathHelper.fromChoreoPath("Tag14").getPath().getStartingDifferentialPose()
        addCommands(
                Commands.startEnd(drivetrain::blockTags,drivetrain::unblockTags),
                Commands.sequence(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ElevatorSetpointCommand(elevator, ElevatorState.BOTTOM),
                                new StageTrampCommand(tramp, indexer).onlyIf(indexer::isNoteStaged)
                        ),
                        new SequentialCommandGroup(
//                                pathHelper.resetToInitialPose(drivetrain),
                                new ProxyCommand(() -> climbPath(drivetrain::getPose))//                                pathHelper.getCommand(drivetrain)
                        )
                        //TODO: Put whatever drive/alignment command we plan on using here
                ),
                Commands.waitSeconds(1),
//                new ElevatorSetpointCommand(elevator, ElevatorState.CLIMB),
                Commands.waitSeconds(1)
                //TODO: Put whatever drive/alignment command we plan on using here
//                new ElevatorSetpointCommand(elevator, ElevatorState.BOTTOM)
//                hooks.deployHooksCommand() //TODO: Deploy hooks
                )
        );
    }

    private Command climbPath(Supplier<Pose2d> position){
        double far = position.get().getTranslation().getDistance(
                AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.CLIMB_FAR).getTranslation().toTranslation2d());
        double amp = position.get().getTranslation().getDistance(
                AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.CLIMB_AMP).getTranslation().toTranslation2d());
        double source = position.get().getTranslation().getDistance(
                AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.CLIMB_SOURCE).getTranslation().toTranslation2d());
        Command pathCmd;
        if(far < amp && far < source){
            pathCmd= AutoBuilder.pathfindThenFollowPath(PathHelper.fromChoreoPath("Tag14").getPath(), STAGE_PATH_CONSTRAINTS);
        }
        else if(amp < source){
            pathCmd= AutoBuilder.pathfindThenFollowPath(PathHelper.fromChoreoPath("Tag15").getPath(), STAGE_PATH_CONSTRAINTS);
        }
        else{
            pathCmd= AutoBuilder.pathfindThenFollowPath(PathHelper.fromChoreoPath("Tag16").getPath(), STAGE_PATH_CONSTRAINTS);
        }
        return Commands.runOnce(drivetrain::blockTags).andThen(pathCmd).andThen(Commands.runOnce(drivetrain::unblockTags));
    }
}
