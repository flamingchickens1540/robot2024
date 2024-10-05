package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.shooter.ShooterSetpoint;
import org.team1540.robot2024.util.vision.AprilTagsCrescendo;

import java.util.function.Supplier;

public class OverStageShootPrepare extends SequentialCommandGroup {
    public OverStageShootPrepare(Drivetrain drivetrain, Shooter shooter) {
        this(drivetrain::getPose, shooter);
    }

    public OverStageShootPrepare(Supplier<Pose2d> positionSupplier, Shooter shooter, double leftSetpoint, double rightSetpoint) {
        addCommands(
                new PrepareShooterCommand(shooter, () -> new ShooterSetpoint(
                        Rotation2d.fromRadians(
                                Math.atan2(Constants.Targeting.STAGE_MAX_HEIGHT + 3.5 - Constants.Shooter.Pivot.PIVOT_HEIGHT,
                                        positionSupplier.get().getTranslation().getDistance(
                                        AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.CLIMB_FAR).toPose2d().getTranslation()
                                        )
                                )
                        ).minus(Constants.Shooter.Pivot.REAL_ZEROED_ANGLE),
                        leftSetpoint, rightSetpoint)
                )
        );
    }

    public OverStageShootPrepare(Supplier<Pose2d> positionSupplier, Shooter shooter){
        this(positionSupplier, shooter, 3500, 3500);
    }
}
