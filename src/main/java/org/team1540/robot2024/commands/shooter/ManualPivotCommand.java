package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.math.JoystickUtils;

public class ManualPivotCommand extends Command {
    private final Shooter shooter;
    private final CommandXboxController controller;

    public ManualPivotCommand(Shooter shooter, CommandXboxController controller) {
        this.shooter = shooter;
        this.controller = controller;
        addRequirements(shooter);
    }

    @Override
    public void execute() {

//        shooter.setPivotVolts(JoystickUtils.smartDeadzone(controller.getLeftY()*8, 0.1));

        shooter.holdPivotPosition();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheels();
        shooter.stopPivot();
    }
}
