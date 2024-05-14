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
    private static final double A = -100.0;
    private static final double B = 53.4984;
    private static final double C = 46.8837;
    private static final double D = 157.311;

    public AutoShootPrepare(Drivetrain drivetrain, Shooter shooter) {
        this(drivetrain::getPose, shooter);
    }

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
            )
        );
    }

    public AutoShootPrepare(Supplier<Pose2d> positionSupplier, Shooter shooter){
        this(positionSupplier, shooter, Constants.Shooter.Flywheels.LEFT_RPM, Constants.Shooter.Flywheels.RIGHT_RPM);
    }
}
