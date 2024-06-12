package org.team1540.robot2024.util.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2024.commands.drivetrain.DriveWithTargetingCommand;
import org.team1540.robot2024.commands.indexer.ContinuousIntakeCommand;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;

import java.util.ArrayList;
import java.util.List;

public class AutoCommand extends SequentialCommandGroup {
    private String name;
    private boolean isResetting = false;
    private Pose2d initialPose = null;

    private final List<PathHelper> paths = new ArrayList<>();

    private int pathIndex = 0;

    public AutoCommand(String name){
        this.name = name;
    }

    public AutoCommand(String name, Command command){
        this(name);
        addCommands(command);
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public boolean getIsResetting() {
        return this.isResetting;
    }

    public Pose2d getInitialPose() {
        return initialPose;
    }


    public void addPath(PathHelper... paths){
        for (PathHelper path: paths) {
            this.paths.add(path);
        }
        if(!this.paths.isEmpty() && this.paths.size() == paths.length){
            isResetting = this.paths.get(0).isResetting;
            initialPose = this.paths.get(0).initialPose;
        }
    }

    public List<PathHelper> getPaths() {
        return paths;
    }

    public PathHelper getPath(int pathIndex){
        return paths.get(pathIndex);
    }

    public PathHelper getNextPath(){
        return getPath(pathIndex++);
    }

    public int getPathIndex(){
        return pathIndex;
    }

    public void setPathIndex(int index){
        this.pathIndex = index;
    }

    /**
     * Follows the path while intaking, then stops and runs the feeder
     * @param drivetrain
     * @param indexer
     * @param pathIndex
     * @return the commands to run in the segment
     */
    protected Command createSegmentSequence(Drivetrain drivetrain, Shooter shooter, Indexer indexer, int pathIndex, boolean shouldZeroCancoder, boolean shouldRealignYaw, boolean shouldResetOdometry, double extraPreShotWait, double postShotWaitReduction, boolean shouldSkipIfEmpty) {
        return Commands.sequence(
                Commands.deadline(
                        getPath(pathIndex).getCommand(drivetrain),
                        new ContinuousIntakeCommand(indexer, 1)
                ),
                drivetrain.commandStop(),
                Commands.sequence(
                        Commands.waitSeconds(0.25),
                        new InstantCommand(shooter::zeroPivotToCancoder)
                ).onlyIf(()->shouldZeroCancoder),
                drivetrain.commandCopyVisionPose().onlyIf(()->shouldResetOdometry),
                Commands.parallel(
                        new DriveWithTargetingCommand(drivetrain, null).withTimeout(0.4).onlyIf(()->shouldRealignYaw),
                        Commands.sequence(
//                                Commands.waitUntil(()->drivetrain.getPose().getRotation().minus(drivetrain.getTargetPose().getRotation()).getDegrees()<10).onlyIf(()->shouldRealignYaw),
                                new ParallelDeadlineGroup(
                                        Commands.sequence(
                                                Commands.waitUntil(indexer::isNoteStagedBack),
                                                Commands.waitSeconds(extraPreShotWait),
                                                Commands.waitUntil(()->!indexer.isNoteStagedBack()),
                                                Commands.waitSeconds(0.2 - postShotWaitReduction)
                                        ).withTimeout(1+extraPreShotWait),
                                        IntakeAndFeed.withDefaults(indexer)
                                )
                        )
                ).unless(() -> shouldSkipIfEmpty && !indexer.isNoteStagedBack())
        );
    }

    protected Command createSegmentSequence(Drivetrain drivetrain, Shooter shooter, Indexer indexer, int pathIndex, boolean shouldZeroCancoder, boolean shouldRealignYaw, boolean shouldResetOdometry, double extraPreShotWait, double postShotWaitReduction) {
        return createSegmentSequence(drivetrain, shooter, indexer, pathIndex, shouldZeroCancoder, shouldRealignYaw, shouldResetOdometry, extraPreShotWait, postShotWaitReduction, false);
    }
    protected Command createSegmentSequence(Drivetrain drivetrain, Shooter shooter, Indexer indexer, int pathIndex, boolean shouldZeroCancoder, boolean shouldRealignYaw, boolean shouldResetOdometry, double extraPreShotWait) {
        return createSegmentSequence(drivetrain, shooter, indexer, pathIndex, shouldZeroCancoder, shouldRealignYaw, shouldResetOdometry, extraPreShotWait, 0, false);
    }

    protected Command createSegmentSequence(Drivetrain drivetrain, Shooter shooter, Indexer indexer, int pathIndex){
        return createSegmentSequence(drivetrain, shooter, indexer, pathIndex, false, true, true,0);
    }

    protected Command createSegmentSequence(Drivetrain drivetrain, Shooter shooter, Indexer indexer, int pathIndex, boolean shouldZeroCancoder, boolean shouldRealignYaw, boolean shouldResetOdometry){
        return createSegmentSequence(drivetrain, shooter, indexer, pathIndex, shouldZeroCancoder, shouldRealignYaw, shouldResetOdometry,0);
    }

    protected Command createCancoderSegmentSequence(Drivetrain drivetrain, Shooter shooter, Indexer indexer, int pathIndex) {
        return createSegmentSequence(drivetrain, shooter, indexer, pathIndex, true, true, true,0 );
    }

    public List<Pose2d> toTrajectory() {
        List<Pose2d> output = new ArrayList<>();
        for (PathHelper path : getPaths()) {
            output.addAll(path.path.getPathPoses());
        }
        return output;
    }
}
