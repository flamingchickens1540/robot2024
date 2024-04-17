package org.team1540.robot2024.commands.shooter;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.shooter.ShooterSetpoint;
import org.team1540.robot2024.util.vision.AprilTagsCrescendo;
import org.team1540.robot2024.util.vision.FlipUtil;

import java.util.function.Supplier;

public class CounterShufflePrepare extends SequentialCommandGroup {
    public CounterShufflePrepare(Drivetrain drivetrain, Shooter shooter) {
        this(drivetrain::getPose, shooter);
    }

    public CounterShufflePrepare(Supplier<Pose2d> positionSupplier, Shooter shooter, double leftSetpoint, double rightSetpoint) {
        addCommands(
                new PrepareShooterCommand(shooter, () -> new ShooterSetpoint(
                        Rotation2d.fromRadians(
                                Math.atan2(Constants.Targeting.STAGE_MAX_HEIGHT + 1.5 - Constants.Shooter.Pivot.PIVOT_HEIGHT,
                                        positionSupplier.get().getTranslation().getDistance(
                                        AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.CLIMB_FAR, DriverStation.Alliance.Blue).toPose2d().getTranslation().plus(new Translation2d(FlipUtil.flipIfRed(-2.277226), 0))
                                        )
                                )
                        ).minus(Constants.Shooter.Pivot.REAL_ZEROED_ANGLE),
                        leftSetpoint, rightSetpoint)
                )
        );
    }

    public CounterShufflePrepare(Supplier<Pose2d> positionSupplier, Shooter shooter){
        this(positionSupplier, shooter, 3500, 3500);
    }
}
