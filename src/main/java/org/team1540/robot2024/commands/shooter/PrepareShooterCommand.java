package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.shooter.Shooter;

public class PrepareShooterCommand extends Command {
    private final Shooter shooter;

    public PrepareShooterCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
    @Override
    public void execute() {
        // TODO: Make this dynamically update based on estimated pose
        shooter.setFlywheelSpeeds(6000, 5000);
        shooter.setPivotPosition(Rotation2d.fromDegrees(30));
    }

    @Override
    public boolean isFinished() {
        return true;
//        return shooter.areFlywheelsSpunUp() && shooter.isPivotAtSetpoint();
//        return shooter.areFlywheelsSpunUp();
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted){
            shooter.stopFlywheels();
        }
    }
}
