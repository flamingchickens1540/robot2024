package org.team1540.robot2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class AutoCommand extends SequentialCommandGroup {
    private String name;
    private boolean isResetting = false;
    private Pose2d initialPose = null;

    private List<PathHelper> paths = new ArrayList<>();

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

    public String getName() { return name;}
    public boolean getIsResetting() {
        return this.isResetting;
    }
    public Pose2d getInitialPose() {
        return initialPose;
    }

    public void addPath(PathHelper... paths){
        this.paths.addAll(Arrays.stream(paths).toList());
        if(!this.paths.isEmpty() && this.paths.size() == Arrays.stream(paths).count()){
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
}
