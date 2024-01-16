package org.team1540.robot2024.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
        double xPercent = xLimiter.calculate(-controller.getLeftY());
        double yPercent = yLimiter.calculate(-controller.getLeftX());
        double rotPercent = rotLimiter.calculate(-controller.getRightX());

        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(xPercent, yPercent), 0.1);
        Rotation2d linearDirection = new Rotation2d(xPercent, yPercent);
        double omega = MathUtil.applyDeadband(rotPercent, 0.1);

        // Calculate new linear velocity
        Translation2d linearVelocity =
                new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

        // Convert to field relative speeds & send command
        drivetrain.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.getX() * drivetrain.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * drivetrain.getMaxLinearSpeedMetersPerSec(),
                        omega * drivetrain.getMaxAngularSpeedRadPerSec(),
                        drivetrain.getRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
