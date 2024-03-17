package org.team1540.robot2024.commands.drivetrain;

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
        double xPercent   = JoystickUtils.squaredSmartDeadzone(-controller.getLeftY(), deadzone) * (Constants.IS_COMPETITION_ROBOT ? 1 : -1);
        double yPercent   = JoystickUtils.squaredSmartDeadzone(-controller.getLeftX(), deadzone) * (Constants.IS_COMPETITION_ROBOT ? 1 : -1);
        double rotPercent = JoystickUtils.squaredSmartDeadzone(-controller.getRightX(), deadzone) * (Constants.IS_COMPETITION_ROBOT ? 1 : -1);

        drivetrain.drivePercent(xPercent, yPercent, rotPercent, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
