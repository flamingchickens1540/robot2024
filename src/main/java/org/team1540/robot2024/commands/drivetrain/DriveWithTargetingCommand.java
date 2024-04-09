package org.team1540.robot2024.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.util.LoggedTunableNumber;
import org.team1540.robot2024.util.math.JoystickUtils;

import java.util.function.Supplier;

import static org.team1540.robot2024.Constants.Targeting.*;

public class DriveWithTargetingCommand extends Command {
    private final Drivetrain drivetrain;
    private final XboxController controller;

    private final PIDController rotController = new PIDController(ROT_KP, ROT_KI, ROT_KD);

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Targeting/ROT_KP", ROT_KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Targeting/ROT_KI", ROT_KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Targeting/ROT_KD", ROT_KD);

    private boolean isFlipped;

    private final Supplier<Pose2d> target;

    private final double deadzone = 0.03;

    public DriveWithTargetingCommand(Drivetrain drivetrain, XboxController controller){
        this(drivetrain, controller, Constants.Targeting::getSpeakerPose);
    }
    public DriveWithTargetingCommand(Drivetrain drivetrain, XboxController controller, Supplier<Pose2d> target) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.target = target;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rotController.reset();
        isFlipped = DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red;
    }

    @Override
    public void execute() {
        Rotation2d targetRot =
                drivetrain.getPose()
                        .minus(target.get()).getTranslation().getAngle()
                        .rotateBy(isFlipped ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0));
        drivetrain.setTargetPose(new Pose2d(drivetrain.getPose().getTranslation(), targetRot));
        Logger.recordOutput("Targeting/rotError", targetRot.minus(drivetrain.getRotation()));
        Logger.recordOutput("Targeting/target", target.get());

        double linearMagnitude = 0;
        Rotation2d linearDirection = new Rotation2d();
        if(controller != null){
            double xPercent   = -controller.getLeftY() * (Constants.IS_COMPETITION_ROBOT ? 1 : -1);
            double yPercent   = -controller.getLeftX() * (Constants.IS_COMPETITION_ROBOT ? 1 : -1);
            linearMagnitude = JoystickUtils.smartDeadzone(Math.hypot(xPercent, yPercent), deadzone);
            linearDirection = new Rotation2d(xPercent, yPercent);
        }
        double rotPercent = rotController.calculate(drivetrain.getRotation().getRadians(), targetRot.getRadians());
        drivetrain.drivePercent(linearMagnitude, linearDirection, rotPercent, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
