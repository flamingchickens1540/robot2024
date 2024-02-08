package org.team1540.robot2024.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.subsystems.drive.Drivetrain;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class AutoCommand extends SequentialCommandGroup {
    private String name;
    private boolean isResetting = false;
    private Pose2d initialPose = null;

    private List<PathPlannerHelper> paths = new ArrayList<>();

    private int index = 0;

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

    public void addPath(PathPlannerHelper... paths){
        this.paths.addAll(Arrays.stream(paths).toList());
        if(this.paths.size() != 0 && this.paths.size() == Arrays.stream(paths).count()){
            isResetting = this.paths.get(0).isResetting;
            initialPose = this.paths.get(0).initialPose;
        }
    }

    public List<PathPlannerHelper> getPaths() {
        return paths;
    }

    public PathPlannerHelper getPath(int index){
        return paths.get(index);
    }
    public PathPlannerHelper getNextPath(){
        return getPath(index++);
    }
    public int getIndex(){
        return index;
    }
    public void setIndex(int index){
        this.index = index;
    }
}
