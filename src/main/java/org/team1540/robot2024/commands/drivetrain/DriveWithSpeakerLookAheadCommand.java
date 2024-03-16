package org.team1540.robot2024.commands.drivetrain;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.util.LoggedTunableNumber;

import static org.team1540.robot2024.Constants.Targeting.*;

public class DriveWithSpeakerLookAheadCommand extends Command {
    private final Drivetrain drivetrain;
    private final XboxController controller;

    private final PIDController rotController = new PIDController(ROT_KP, ROT_KI, ROT_KD);

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Targeting/ROT_KP", ROT_KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Targeting/ROT_KI", ROT_KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Targeting/ROT_KD", ROT_KD);

    private boolean isFlipped;
    private Pose2d speakerPose;
    private Pose3d speakerPose3d;

    public DriveWithSpeakerLookAheadCommand(Drivetrain drivetrain, XboxController controller) {
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
        speakerPose3d = new Pose3d(speakerPose.getX(), speakerPose.getY(), SPEAKER_CENTER_HEIGHT, new Rotation3d());
    }

    @Override
    public void execute() {
//        if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
//            rotController.setPID(kP.get(), kI.get(), kD.get());
//        }
        double flightDistance = speakerPose3d.getTranslation().getDistance(new Translation3d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), Constants.Shooter.Pivot.PIVOT_HEIGHT));
        double NOTE_VELOCITY = 32;
        double time = flightDistance/NOTE_VELOCITY;
        Translation2d modifier = new Translation2d(drivetrain.getChassisSpeeds().vxMetersPerSecond * time,drivetrain.getChassisSpeeds().vyMetersPerSecond*time);
        Pose2d drivetrainPose = drivetrain.getPose().plus(new Transform2d(modifier.getX(), modifier.getY(), new Rotation2d()));
        Logger.recordOutput("Odometry/PredictedPosition", drivetrainPose);
        Rotation2d targetRot =
                drivetrainPose
                        .minus(speakerPose).getTranslation().getAngle()
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
