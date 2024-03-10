package org.team1540.robot2024.commands.drivetrain;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.util.LoggedTunableNumber;

import static org.team1540.robot2024.Constants.Targeting.*;

public class HoldRotation extends Command {
    private final Drivetrain drivetrain;
    private final CommandXboxController controller;

    private final PIDController rotController = new PIDController(ROT_KP, ROT_KI, ROT_KD);

    private boolean isFlipped;
    private Rotation2d targetRotation;

    public HoldRotation(Drivetrain drivetrain, CommandXboxController controller, Rotation2d targetRotation) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.targetRotation = targetRotation;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rotController.reset();

        isFlipped =
                DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    @Override
    public void execute() {
        Logger.recordOutput("Targeting/setpointPose", new Pose2d(drivetrain.getPose().getTranslation(), targetRotation));
        double xPercent = MathUtil.applyDeadband((-controller.getLeftY()), 0.1);
        double yPercent = MathUtil.applyDeadband((-controller.getLeftX()), 0.1);
        double rotPercent = rotController.calculate(drivetrain.getRotation().getRadians(), targetRotation.getRadians());
        drivetrain.drivePercent(xPercent, yPercent, rotPercent, isFlipped);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
