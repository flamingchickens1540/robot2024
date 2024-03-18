package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2024.subsystems.shooter.Shooter;

public class HoldPivotCommand extends Command {
    private final Shooter shooter;

    public HoldPivotCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setPivotVolts(4);
    }
    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheels();
        shooter.stopPivot();
    }
}
