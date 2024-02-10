package org.team1540.robot2024.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.subsystems.drive.Drivetrain;

import java.util.function.BooleanSupplier;

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
        Rotation2d rotation = path.getPoint(0).rotationTarget == null ? new Rotation2d() : path.getPoint(0).rotationTarget.getTarget();
        this.initialPose = new Pose2d(path.getPoint(0).position, rotation);
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

    public Command getCommand(boolean shouldRealign){
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));
        BooleanSupplier shouldFlip = ()->(DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red);
        Command command = new ConditionalCommand(
                AutoBuilder.pathfindThenFollowPath(path,constraints),
                AutoBuilder.followPath(path),
                ()->drivetrain.getPose().getTranslation().getDistance((shouldFlip.getAsBoolean() ? GeometryUtil.flipFieldPose(initialPose) : initialPose).getTranslation()) > 1 && shouldRealign); //TODO tune this distance
        Command resetCommand = new InstantCommand(() -> drivetrain.setPose(shouldFlip.getAsBoolean() ? GeometryUtil.flipFieldPose(initialPose) : initialPose));
        if(isResetting)return resetCommand.andThen(command);
        else return command;
    }
    public Command getCommand(){
       return getCommand(false);
    }
}
