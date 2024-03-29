package org.team1540.robot2024.commands.drivetrain;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.util.LoggedTunableNumber;

import static org.team1540.robot2024.Constants.Targeting.*;

public class DriveWithSpeakerTargetingCommand extends Command {
    private final Drivetrain drivetrain;
    private final XboxController controller;

    private final PIDController rotController = new PIDController(ROT_KP, ROT_KI, ROT_KD);

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Targeting/ROT_KP", ROT_KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Targeting/ROT_KI", ROT_KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Targeting/ROT_KD", ROT_KD);

    private boolean isFlipped;
    private Pose2d speakerPose;

    public DriveWithSpeakerTargetingCommand(Drivetrain drivetrain, XboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rotController.reset();
        isFlipped = DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red;
        speakerPose = isFlipped ? GeometryUtil.flipFieldPose(SPEAKER_POSE) : SPEAKER_POSE;
    }

    @Override
    public void execute() {
        if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            rotController.setPID(kP.get(), kI.get(), kD.get());
        }

        Rotation2d targetRot =
                drivetrain.getPose().minus(speakerPose).getTranslation().getAngle()
                        .rotateBy(isFlipped ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0));
        Logger.recordOutput("Targeting/setpointPose", new Pose2d(drivetrain.getPose().getTranslation(), targetRot));
        Logger.recordOutput("Targeting/rotErrorDegrees", Math.abs(targetRot.minus(drivetrain.getRotation()).getDegrees()));

        double xPercent = MathUtil.applyDeadband((-controller.getLeftY()), 0.1);
        double yPercent = MathUtil.applyDeadband((-controller.getLeftX()), 0.1);
        double rotPercent = rotController.calculate(drivetrain.getRotation().getRadians(), targetRot.getRadians());
        drivetrain.drivePercent(xPercent, yPercent, rotPercent, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
