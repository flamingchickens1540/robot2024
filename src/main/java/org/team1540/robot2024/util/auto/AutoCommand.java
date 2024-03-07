package org.team1540.robot2024.util.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;

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
    protected Command createSegmentSequence(Drivetrain drivetrain, Indexer indexer, int pathIndex) {
        return Commands.sequence(
                Commands.deadline(
                        getPath(pathIndex).getCommand(drivetrain),
                        new IntakeCommand(indexer, () -> false, 1)
                ),
                drivetrain.commandStop(),
                IntakeAndFeed.withDefaults(indexer).withTimeout(0.5)
        );
    }

    public List<Pose2d> toTrajectory() {
        List<Pose2d> output = new ArrayList<>();
        for (PathHelper path : getPaths()) {
            output.addAll(path.path.getPathPoses());
        }
        return output;
    }
}
