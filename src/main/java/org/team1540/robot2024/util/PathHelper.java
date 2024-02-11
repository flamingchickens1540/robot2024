package org.team1540.robot2024.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class PathHelper {
    private static final PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720)
    );

    boolean isResetting;
    Pose2d initialPose;
    String pathname;
    boolean isChoreo;
    boolean canFlip;
    PathPlannerPath path;


    public static PathHelper fromChoreoPath(String pathname) {
        return new PathHelper(pathname, true, false, true);
    }

    public static PathHelper fromChoreoPath(String pathname, boolean shouldReset, boolean canFlip) {
        return new PathHelper(pathname, true, shouldReset, canFlip);
    }

    public static PathHelper fromPathPlannerPath(String pathname) {
        return new PathHelper(pathname, false, false, true);
    }

    public static PathHelper fromPathPlannerPath(String pathname, boolean shouldReset, boolean canFlip) {
        return new PathHelper(pathname, false, shouldReset, canFlip);
    }


    private PathHelper(String pathname, boolean isChoreo, boolean shouldReset, boolean canFlip) {
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

    public PathPlannerPath getPath() {
        return path;
    }

    public Command getCommand(Drivetrain drivetrain, boolean shouldRealign) {
        BooleanSupplier shouldFlip = () -> DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red;
        Supplier<Pose2d> flippedInitialPose = () -> shouldFlip.getAsBoolean() ? GeometryUtil.flipFieldPose(initialPose) : initialPose;
        Command command = new ConditionalCommand(
                AutoBuilder.pathfindThenFollowPath(path, constraints),
                AutoBuilder.followPath(path),
                () -> drivetrain.getPose().getTranslation().getDistance((flippedInitialPose.get()).getTranslation()) > 1 && shouldRealign); //TODO tune this distance
        Command resetCommand = new InstantCommand(() -> drivetrain.setPose(flippedInitialPose.get()));
        return isResetting ? resetCommand.andThen(command) : command;
    }

    public Command getCommand(Drivetrain drivetrain) {
        return getCommand(drivetrain, false);
    }
}
