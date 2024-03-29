package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.shooter.ShooterSetpoint;
import org.team1540.robot2024.util.vision.AprilTagsCrescendo;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TuneShooterCommand extends ParallelCommandGroup {
    private final LoggedDashboardNumber leftFlywheelSetpoint = new LoggedDashboardNumber("Shooter/Flywheels/leftSetpoint", 7000);
    private final LoggedDashboardNumber rightFlywheelSetpoint = new LoggedDashboardNumber("Shooter/Flywheels/rightSetpoint", 3000);
    private final LoggedDashboardNumber angleSetpoint = new LoggedDashboardNumber("Shooter/Pivot/angleSetpoint", 15);

    private final DoubleSupplier shooterLeftSupplier;
    private final DoubleSupplier shooterRightSupplier;
    private final Supplier<Rotation2d> pivotRotationSupplier;

    private int shotNum = 0;
    public TuneShooterCommand(Shooter shooter, Indexer indexer, Supplier<Pose2d> poseSupplier) {
        shooterLeftSupplier = shooter::getLeftFlywheelSetpointRPM;
        shooterRightSupplier = shooter::getRightFlywheelSetpointRPM;
        pivotRotationSupplier = shooter::getPivotPosition;

        addCommands(
                new IntakeAndFeed(indexer, () -> 1, () -> 1),
                new PrepareShooterCommand(shooter, () ->
                        new ShooterSetpoint(
                                Rotation2d.fromDegrees(angleSetpoint.get()), leftFlywheelSetpoint.get(), rightFlywheelSetpoint.get()
                        )),
                Commands.sequence(
                        Commands.waitUntil(indexer::isNoteStaged),
                        Commands.waitUntil(()->!indexer.isNoteStaged()),
                        Commands.waitSeconds(1),
                        Commands.runOnce(()->shotNum += 1),
                        Commands.print("Shot Number: " + shotNum + " Angle Degrees Setpoint: " + pivotRotationSupplier.get() + " Left RPM Setpoint: " + shooterLeftSupplier.getAsDouble() + " Right RPM Setpoint: " + shooterRightSupplier.getAsDouble() +
                                " Angle Degrees: " + shooter.getPivotPosition().getDegrees() + " Left RPM: " + shooter.getLeftFlywheelSpeed() + " Right RPM: " + shooter.getRightFlywheelSpeed() +
                                " Distance: " + poseSupplier.get().getTranslation().getDistance(AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.SPEAKER_CENTER).toPose2d().getTranslation()))
                ).repeatedly()
        );
        addRequirements(shooter, indexer);
    }
}
