package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.team1540.robot2024.subsystems.shooter.Shooter;

public class PrepareShooterCommand extends Command {
    private final Shooter shooter;
    private final LoggedDashboardNumber leftFlywheelSetpoint = new LoggedDashboardNumber("Shooter/Flywheels/leftSetpoint", 3200);
    private final LoggedDashboardNumber rightFlywheelSetpoint = new LoggedDashboardNumber("Shooter/Flywheels/rightSetpoint", 2500);

    public PrepareShooterCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
    @Override
    public void execute() {
        // TODO: Make this dynamically update based on estimated pose
        shooter.setFlywheelSpeeds(leftFlywheelSetpoint.get(), rightFlywheelSetpoint.get());
//        shooter.setPivotPosition(Rotation2d.fromDegrees(30));
    }

    @Override
    public boolean isFinished() {
        return false;
//        return shooter.areFlywheelsSpunUp() && shooter.isPivotAtSetpoint(); TODO make this terminate properly so it works
//        return shooter.areFlywheelsSpunUp();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheels();
    }
}
