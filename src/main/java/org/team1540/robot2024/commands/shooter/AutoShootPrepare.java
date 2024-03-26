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

public class AutoShootPrepare extends SequentialCommandGroup {
    public AutoShootPrepare(Drivetrain drivetrain, Shooter shooter) {
        this(drivetrain::getPose, shooter);
    }
    double A = -0.6953;
    double B = 0.8702;
    double C = -0.4942;
    double D = 1.491;

    public AutoShootPrepare(Supplier<Pose2d> positionSupplier, Shooter shooter, double leftSetpoint, double rightSetpoint) {
        addCommands(
            new PrepareShooterCommand(shooter, () -> //new ShooterSetpoint(
//                    Rotation2d.fromRadians(
//                            Math.atan2(Constants.Targeting.SPEAKER_CENTER_HEIGHT - Constants.Shooter.Pivot.PIVOT_HEIGHT,
//                                    positionSupplier.get().getTranslation().getDistance(
//                                    AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.SPEAKER_CENTER).toPose2d().getTranslation()
//                                    )
//                            )
//                    ).minus(Constants.Shooter.Pivot.REAL_ZEROED_ANGLE),
//                    8000, 6000)
//                    shooter.lerp.get(positionSupplier.get().getTranslation().getDistance(AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.SPEAKER_CENTER).toPose2d().getTranslation()))
                    new ShooterSetpoint(
                            Rotation2d.fromRadians(
                                    A * Math.atan(B * (positionSupplier.get().getTranslation().getDistance(AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.SPEAKER_CENTER).toPose2d().getTranslation())) + C) + D
                            ),
                            leftSetpoint,
                            rightSetpoint
                    )
            )
        );
    }

    public AutoShootPrepare(Supplier<Pose2d> positionSupplier, Shooter shooter){
        this(positionSupplier, shooter, 7000, 3000);
    }
}
