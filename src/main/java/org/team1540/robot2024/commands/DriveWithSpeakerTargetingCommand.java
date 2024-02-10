package org.team1540.robot2024.commands;

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

public class DriveWithSpeakerTargetingCommand extends Command {
    private final Drivetrain drivetrain;
    private final CommandXboxController controller;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(2);
    private final PIDController rotController = new PIDController(ROT_KP, ROT_KI, ROT_KD);

    private boolean isFlipped;
    private Pose2d speakerPose;

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Targeting/ROT_KP", ROT_KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Targeting/ROT_KI", ROT_KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Targeting/ROT_KD", ROT_KD);

    public DriveWithSpeakerTargetingCommand(Drivetrain drivetrain, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        xLimiter.reset(0);
        yLimiter.reset(0);
        rotController.reset();

        isFlipped =
                DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        speakerPose = isFlipped ? GeometryUtil.flipFieldPose(SPEAKER_POSE) : SPEAKER_POSE;
    }

    @Override
    public void execute() {
        if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            rotController.setPID(kP.get(), kI.get(), kD.get());
        }

        Rotation2d targetRot =
                drivetrain.getPose().minus(speakerPose).getTranslation().getAngle()
                        .rotateBy(isFlipped ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180));
        Logger.recordOutput("Targeting/setpointPose", new Pose2d(drivetrain.getPose().getTranslation(), targetRot));

        double xPercent = MathUtil.applyDeadband(xLimiter.calculate(-controller.getLeftY()), 0.1);
        double yPercent = MathUtil.applyDeadband(yLimiter.calculate(-controller.getLeftX()), 0.1);
        double rotPercent = rotController.calculate(drivetrain.getRotation().getRadians(), targetRot.getRadians());
        drivetrain.drivePercent(xPercent, yPercent, rotPercent, isFlipped);

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
