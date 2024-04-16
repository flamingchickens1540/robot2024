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
    public
    AutoShootPrepare(Drivetrain drivetrain, Shooter shooter) {
        this(drivetrain::getPose, shooter);
    }
    double A = -100.0;
    double B = 53.4984;
    double C = 46.8837;
    double D = 157.311;

    public AutoShootPrepare(Supplier<Pose2d> positionSupplier, Shooter shooter, double leftSetpoint, double rightSetpoint) {
        addCommands(
            new PrepareShooterCommand(shooter, () ->
                    new ShooterSetpoint(
                            Rotation2d.fromRadians(
                                    A * Math.atan(B * (positionSupplier.get().getTranslation().getDistance(AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.SPEAKER_CENTER).toPose2d().getTranslation())) + C) + D
                            ),
                            leftSetpoint,
                            rightSetpoint
                    )
//                    shooter.lerp.get(positionSupplier.get().getTranslation().getDistance(AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.SPEAKER_CENTER).toPose2d().getTranslation()))
            )
        );
    }

    public AutoShootPrepare(Supplier<Pose2d> positionSupplier, Shooter shooter){
        this(positionSupplier, shooter, Constants.Shooter.Flywheels.LEFT_RPM, Constants.Shooter.Flywheels.RIGHT_RPM);
    }
}
