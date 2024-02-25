package org.team1540.robot2024.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2024.subsystems.drive.Drivetrain;

public class SwerveDriveCommand extends Command {
    private final Drivetrain drivetrain;
    private final CommandXboxController controller;

    public SwerveDriveCommand(Drivetrain drivetrain, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // TODO: Try using a hole-less deadband here and see what William thinks
        double xPercent = MathUtil.applyDeadband((-controller.getLeftY()), 0.1);
        double yPercent = MathUtil.applyDeadband((-controller.getLeftX()), 0.1);
        double rotPercent = MathUtil.applyDeadband((-controller.getRightX()), 0.1);

        drivetrain.drivePercent(xPercent, yPercent, rotPercent, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
