package org.team1540.robot2024.util.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2024.commands.drivetrain.DriveWithTargetingCommand;
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
    protected Command createSegmentSequence(Drivetrain drivetrain, Shooter shooter, Indexer indexer, int pathIndex, boolean shouldZeroCancoder, boolean shouldRealignYaw, boolean shouldResetOdometry, double extraPreShotWait) {
        return Commands.sequence(
                Commands.deadline(
                        getPath(pathIndex).getCommand(drivetrain),
                        new IntakeCommand(indexer, () -> false, 1)
                ),
                drivetrain.commandStop(),
                Commands.sequence(
                        Commands.waitSeconds(0.25),
                        new InstantCommand(shooter::zeroPivotToCancoder)
                ).onlyIf(()->shouldZeroCancoder),
                drivetrain.commandCopyVisionPose().onlyIf(()->shouldResetOdometry),
                Commands.deadline(
                        Commands.deadline(
                                Commands.sequence(
                                        Commands.waitSeconds(extraPreShotWait),
                                        Commands.waitUntil(()->!indexer.isNoteStaged()),
                                        Commands.waitSeconds(0.1)
                                ).withTimeout(1+extraPreShotWait),
                                Commands.waitSeconds(extraPreShotWait).andThen(IntakeAndFeed.withDefaults(indexer))
                        ),
                        new DriveWithTargetingCommand(drivetrain, null).withTimeout(0.4).onlyIf(()->shouldRealignYaw)
                )
        );
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
