package org.team1540.robot2024.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;

public class PathPlannerHelper{
    boolean isResetting = false;
    Pose2d initialPose;
    Drivetrain drivetrain;
    String pathname;
    boolean isChoreo;
    boolean canFlip;
    PathPlannerPath path;

    public PathPlannerHelper(Drivetrain drivetrain, String pathname, boolean isChoreo){
        this(drivetrain, pathname, isChoreo, false, true);
    }

    public PathPlannerHelper(Drivetrain drivetrain, String pathname, boolean isChoreo, boolean shouldReset, boolean canFlip){
        this.drivetrain = drivetrain;
        this.pathname = pathname;
        this.isChoreo = isChoreo;
        this.isResetting = shouldReset;
        this.canFlip = canFlip;
        this.path = isChoreo ? PathPlannerPath.fromChoreoTrajectory(pathname) : PathPlannerPath.fromPathFile(pathname);
        if(canFlip && DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)path = path.flipPath(); //#FIXME Potentially causes problems if flipping is already done not sure how the behavior works
        this.initialPose = new Pose2d(path.getPoint(0).position, path.getPoint(0).rotationTarget.getTarget());
    }

    public boolean getIsResetting() {
        return this.isResetting;
    }

    public Pose2d getInitialPose() {
        return initialPose;
    }

    public PathPlannerPath getPath(){
        return path;
    }

    public Command getCommand(){
        Command command = new ProxyCommand(() -> AutoBuilder.followPath(path));
        if (isResetting) {
            return new InstantCommand(() -> drivetrain.setPose(initialPose)).andThen(command);
        } else {
            return command;
        }
    }
}
