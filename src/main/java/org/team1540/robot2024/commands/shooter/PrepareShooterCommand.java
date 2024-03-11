package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.shooter.ShooterSetpoint;

import java.util.function.Supplier;

public class PrepareShooterCommand extends Command {
    private final Shooter shooter;
    private final Supplier<ShooterSetpoint> setpoint;

    public PrepareShooterCommand(Shooter shooter, Supplier<ShooterSetpoint> setpoint) {
        this.shooter = shooter;
        this.setpoint = setpoint;
        addRequirements(shooter);
    }
    @Override
    public void execute() {
        ShooterSetpoint setpoint = this.setpoint.get();
        shooter.setFlywheelSpeeds(setpoint.leftSetpoint, setpoint.rightSetpoint);
        shooter.setPivotPosition(setpoint.pivot);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheels();
        shooter.stopPivot();
    }
}
