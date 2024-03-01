package org.team1540.robot2024.util.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.subsystems.drive.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class PathHelper {
    final boolean isResetting;
    final Pose2d initialPose;
    final String pathname;
    final boolean isChoreo;
    final boolean canFlip;
    final PathPlannerPath path;

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
        BooleanSupplier shouldFlip = () -> DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red;
        System.out.println(DriverStation.getAlliance().orElse(null) + " " + shouldFlip.getAsBoolean());
        return shouldFlip.getAsBoolean() ? GeometryUtil.flipFieldPose(initialPose) : initialPose;
    }

    public PathPlannerPath getPath() {
        return path;
    }

    public Command getCommand(Drivetrain drivetrain, boolean shouldRealign) {
        BooleanSupplier shouldFlip = () -> DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red;
        Supplier<Pose2d> startingPose = () -> shouldFlip.getAsBoolean() ? GeometryUtil.flipFieldPose(initialPose) : initialPose;
        Command command = new ConditionalCommand(
                AutoBuilder.pathfindThenFollowPath(path, Constants.Auto.PATH_CONSTRAINTS),
                AutoBuilder.followPath(path),
                () -> drivetrain.getPose().getTranslation().getDistance((startingPose.get()).getTranslation()) > 1 && shouldRealign); //TODO tune this distance
        command.addRequirements(drivetrain); //FIXME COuld cause problems if we did things wrong but it shouldnt.
        Command resetCommand = new InstantCommand(() -> drivetrain.setPose(startingPose.get()));
        return (isResetting ? resetCommand.andThen(command) : command);
    }

    public Command resetToInitialPose(Drivetrain drivetrain){
        BooleanSupplier shouldFlip = () -> DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red;
        Supplier<Pose2d> startingPose = () -> shouldFlip.getAsBoolean() ? GeometryUtil.flipFieldPose(initialPose) : initialPose;
        return new ConditionalCommand(
                AutoBuilder.pathfindToPose(startingPose.get(), Constants.Auto.PATH_CONSTRAINTS),
                new InstantCommand(),
                () -> drivetrain.getPose().getTranslation().getDistance((startingPose.get()).getTranslation()) > 1);
    }

    public Command getCommand(Drivetrain drivetrain) {
        return getCommand(drivetrain, false);
    }
}
