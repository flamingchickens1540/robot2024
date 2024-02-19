package org.team1540.robot2024.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2024.subsystems.drive.Drivetrain;

public class SwerveDriveCommand extends Command {
    private final Drivetrain drivetrain;
    private final CommandXboxController controller;

    private boolean isFlipped;

    public SwerveDriveCommand(Drivetrain drivetrain, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        isFlipped =
                DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    @Override
    public void execute() {
        double xPercent = MathUtil.applyDeadband((-controller.getLeftY()), 0.1);
        double yPercent = MathUtil.applyDeadband((-controller.getLeftX()), 0.1);
        double rotPercent = MathUtil.applyDeadband((-controller.getRightX()), 0.1);

        drivetrain.drivePercent(xPercent, yPercent, rotPercent, isFlipped);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}