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

public class AutoShooterPrepare extends SequentialCommandGroup {
    public AutoShooterPrepare(Drivetrain drivetrain, Shooter shooter) {
        this(drivetrain::getPose, shooter);
    }
    public AutoShooterPrepare(Supplier<Pose2d> positionSupplier, Shooter shooter) {
        addCommands(
            new PrepareShooterCommand(shooter, () -> new ShooterSetpoint(
                    Rotation2d.fromRadians(
                            Math.atan2(Constants.Targeting.SPEAKER_CENTER_HEIGHT - Constants.Shooter.Pivot.PIVOT_HEIGHT, positionSupplier.get().getTranslation().getDistance(
                                    AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.SPEAKER_CENTER).toPose2d().getTranslation()
                            ))).minus(Constants.Shooter.Pivot.REAL_ZEROED_ANGLE),
                    8000, 6000)
            )
        );
    }
}
