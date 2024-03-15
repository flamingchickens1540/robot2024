package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.shooter.ShooterSetpoint;

public class TuneShooterCommand extends ParallelCommandGroup {
    private final LoggedDashboardNumber leftFlywheelSetpoint = new LoggedDashboardNumber("Shooter/Flywheels/leftSetpoint", 8000);
    private final LoggedDashboardNumber rightFlywheelSetpoint = new LoggedDashboardNumber("Shooter/Flywheels/rightSetpoint", 6000);
    private final LoggedDashboardNumber angleSetpoint = new LoggedDashboardNumber("Shooter/Pivot/angleSetpoint", 15);

    private int shotNum = 0;
    public TuneShooterCommand(Shooter shooter, Indexer indexer) {

        addCommands(
                new IntakeAndFeed(indexer, ()->1, ()->1),
                new PrepareShooterCommand(shooter, ()->
                        new ShooterSetpoint(
                                Rotation2d.fromDegrees(angleSetpoint.get()), leftFlywheelSetpoint.get(), rightFlywheelSetpoint.get()
                        )),
                Commands.sequence(
                        Commands.waitUntil(indexer::isNoteStaged),
                        Commands.waitUntil(()->!indexer.isNoteStaged()),
                        Commands.waitSeconds(1),
                        Commands.runOnce(()->shotNum += 1),
                        Commands.print("Shot Number: " + shotNum + " Angle Degrees Setpoint: " + shooter.getPivotSetpoint() + " Left RPM Setpoint: " + shooter.getLeftFlywheelSetpointRPM() + " Right RPM Setpoint: " + shooter.getRightFlywheelSetpointRPM() +
                                " Angle Degrees: " + shooter.getPivotPosition().getDegrees() + " Left RPM: " + shooter.getLeftFlywheelSpeed() + " Right RPM: " + shooter.getRightFlywheelSpeed())
                ).repeatedly()
        );
        addRequirements(shooter, indexer);
    }
}
