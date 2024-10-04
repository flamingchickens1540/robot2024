package org.team1540.robot2024.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.util.math.JoystickUtils;

public class SwerveDriveCommand extends Command {
    private final Drivetrain drivetrain;
    private final CommandXboxController controller;
    private static final double deadzone = 0.03;

    public SwerveDriveCommand(Drivetrain drivetrain, CommandXboxController controller) {

        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double xPercent   = -controller.getLeftY() * (Constants.IS_COMPETITION_ROBOT ? 1 : -1);
        double yPercent   = -controller.getLeftX() * (Constants.IS_COMPETITION_ROBOT ? 1 : -1);
        double linearMagnitude = JoystickUtils.smartDeadzone(Math.hypot(xPercent, yPercent), deadzone);
        Rotation2d linearDirection = new Rotation2d(xPercent, yPercent);
        double rotPercent = JoystickUtils.smartDeadzone(-controller.getRightX(), deadzone) * (Constants.IS_COMPETITION_ROBOT ? 1 : -1);

        drivetrain.drivePercent(linearMagnitude*linearMagnitude, linearDirection, rotPercent, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
