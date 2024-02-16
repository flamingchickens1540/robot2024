package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.team1540.robot2024.subsystems.shooter.Shooter;

public class TuneShooterCommand extends Command {
    private final LoggedDashboardNumber leftFlywheelSetpoint = new LoggedDashboardNumber("Shooter/Flywheels/leftSetpoint", 6000);
    private final LoggedDashboardNumber rightFlywheelSetpoint = new LoggedDashboardNumber("Shooter/Flywheels/rightSetpoint", 6000);
    private final LoggedDashboardNumber angleSetpoint = new LoggedDashboardNumber("Shooter/Pivot/angleSetpoint", 30);
    private final Shooter shooter;

    public TuneShooterCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setFlywheelSpeeds(leftFlywheelSetpoint.get(), rightFlywheelSetpoint.get());
        shooter.setPivotPosition(Rotation2d.fromDegrees(angleSetpoint.get()));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheels();
        shooter.stopPivot();
    }
}
