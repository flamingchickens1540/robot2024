package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.commands.drivetrain.DriveWithTargetingCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.shooter.Shooter;

import java.util.function.Supplier;

public class OverStageShootPrepareWithLeadTargeting extends ParallelCommandGroup {
    private static final double LOOK_AHEAD_TIME_SECS = 0.5;

    public OverStageShootPrepareWithLeadTargeting(XboxController controller, Drivetrain drivetrain, Shooter shooter) {
        Supplier<Translation2d> adjustment = () -> {
            ChassisSpeeds robotRelative = drivetrain.getChassisSpeeds();
            ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, drivetrain.getRotation());
            return new Translation2d(
                    fieldRelative.vxMetersPerSecond * LOOK_AHEAD_TIME_SECS,
                    fieldRelative.vyMetersPerSecond * LOOK_AHEAD_TIME_SECS);
        };
        addCommands(
                new DriveWithTargetingCommand(
                        drivetrain,
                        controller,
                        () -> new Pose2d(
                                Constants.Targeting.getShufflePose().getTranslation().plus(adjustment.get().unaryMinus()),
                                Constants.Targeting.getShufflePose().getRotation())),
                new OverStageShootPrepare(
                        () -> new Pose2d(
                                drivetrain.getPose().getTranslation().plus(adjustment.get()),
                                drivetrain.getRotation()),
                        shooter)
        );
    }
}
