package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.shooter.Shooter;

class PrepareShooterCommand extends Command {
    private final Shooter shooter;

    public PrepareShooterCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
    @Override
    public void execute() {
        // TODO: Make this dynamically update based on estimated pose
        shooter.setFlywheelSpeeds(1000, 1000);
        shooter.setPivotPosition(Rotation2d.fromDegrees(30));
    }

    @Override
    public boolean isFinished() {
        return shooter.areFlywheelsSpunUp() && shooter.isPivotAtSetpoint();
    }
}
