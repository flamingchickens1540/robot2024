package org.team1540.robot2024.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.util.math.JoystickUtils;

import java.util.function.Supplier;

public class DriveWithCorrectionCommand extends Command {
    private final Drivetrain drivetrain;
    private final CommandXboxController controller;

    private final Supplier<Double> angleDegrees;
    private static final double deadzone = 0.03;

    private static final double yawScale = 0.5;

    public DriveWithCorrectionCommand(Drivetrain drivetrain, CommandXboxController controller, Supplier<Double> angleDegrees) {

        this.drivetrain = drivetrain;
        this.controller = controller;
        this.angleDegrees = angleDegrees;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double xPercent   = -controller.getLeftY() * (Constants.IS_COMPETITION_ROBOT ? 1 : -1);

        Rotation2d linearDirection = drivetrain.getRotation().rotateBy(Rotation2d.fromDegrees(angleDegrees.get()));

        Logger.recordOutput("Targeting/targetDirection", new Pose2d(
                drivetrain.getPose().getTranslation(),
                linearDirection
        ));

        double rotPercent = JoystickUtils.smartDeadzone(-controller.getRightX(), deadzone) * (Constants.IS_COMPETITION_ROBOT ? 1 : -1);
        drivetrain.drivePercent((xPercent)*(linearDirection.getCos()>0?1:-1), linearDirection, rotPercent, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
