package org.team1540.robot2024.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2024.subsystems.drive.Drivetrain;

public class SwerveDriveCommand extends Command {
    private final Drivetrain drivetrain;
    private final CommandXboxController controller;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    public SwerveDriveCommand(Drivetrain drivetrain, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        xLimiter.reset(0);
        yLimiter.reset(0);
        rotLimiter.reset(0);

    }

    @Override
    public void execute() {
        boolean isFlipped =
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        double xPercent = MathUtil.applyDeadband(xLimiter.calculate(-controller.getLeftY()), 0.1);
        double yPercent = MathUtil.applyDeadband(yLimiter.calculate(-controller.getLeftX()), 0.1);
        double rotPercent = MathUtil.applyDeadband(rotLimiter.calculate(-controller.getRightX()), 0.1);

        drivetrain.drivePercent(xPercent, yPercent, rotPercent, isFlipped);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
