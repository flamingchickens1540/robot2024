package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;

public class TuneShooterCommand extends Command {
    private final LoggedDashboardNumber leftFlywheelSetpoint = new LoggedDashboardNumber("Shooter/Flywheels/leftSetpoint", 6000);
    private final LoggedDashboardNumber rightFlywheelSetpoint = new LoggedDashboardNumber("Shooter/Flywheels/rightSetpoint", 6000);
    private final LoggedDashboardNumber angleSetpoint = new LoggedDashboardNumber("Shooter/Pivot/angleSetpoint", 15);
    private final Shooter shooter;
    private final Indexer indexer;

    public TuneShooterCommand(Shooter shooter, Indexer indexer) {
        this.shooter = shooter;
        this.indexer = indexer;
        addRequirements(shooter, indexer);
    }

    @Override
    public void execute() {
        shooter.setFlywheelSpeeds(leftFlywheelSetpoint.get(), rightFlywheelSetpoint.get());
        shooter.setPivotPosition(Rotation2d.fromDegrees(angleSetpoint.get()));
        indexer.setFeederPercent(1);
        indexer.setIntakePercent(1);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheels();
        shooter.stopPivot();
        indexer.stopAll();
    }
}
