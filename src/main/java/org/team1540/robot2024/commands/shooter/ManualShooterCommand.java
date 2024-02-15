package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.shooter.Shooter;

public class ManualShooterCommand extends Command {
    private final Shooter shooter;
    private final int leftSpeedRPM;
    private final int rightSpeedRPM;

    public ManualShooterCommand(Shooter shooter, int leftSpeedRPM, int rightSpeedRPM) {
        this.shooter = shooter;
        addRequirements(shooter);
        this.leftSpeedRPM = leftSpeedRPM;
        this.rightSpeedRPM = rightSpeedRPM;
    }
    @Override
    public void execute() {
        shooter.setFlywheelSpeeds(leftSpeedRPM, rightSpeedRPM);
    }

    @Override
    public boolean isFinished() {
        return shooter.areFlywheelsSpunUp();
    }
}
