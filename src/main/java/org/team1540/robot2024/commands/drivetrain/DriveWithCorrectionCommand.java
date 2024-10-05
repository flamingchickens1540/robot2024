package org.team1540.robot2024.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.util.math.JoystickUtils;

import java.util.function.Supplier;

import static org.team1540.robot2024.Constants.Targeting.*;

public class DriveWithCorrectionCommand extends Command {
    private final Drivetrain drivetrain;
    private final CommandXboxController controller;

    private final Supplier<Double> angleDegrees;
    private final Supplier<Pose2d> target;
    private static final double deadzone = 0.03;

    private boolean isFlipped;

    private final PIDController rotController = new PIDController(ROT_KP, ROT_KI, ROT_KD);


    public DriveWithCorrectionCommand(Drivetrain drivetrain, CommandXboxController controller, Supplier<Double> angleDegrees, Supplier<Pose2d> target) {

        this.drivetrain = drivetrain;
        this.controller = controller;
        this.angleDegrees = angleDegrees;
        this.target = target;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrain);
    }

    public DriveWithCorrectionCommand(Drivetrain drivetrain, CommandXboxController controller, Supplier<Double> angleDegrees) {
        this(drivetrain, controller, angleDegrees, null);
    }

    @Override
    public void initialize() {
        rotController.reset();
        isFlipped = Constants.Targeting.getFlipped();
    }

    @Override
    public void execute() {
        Rotation2d targetRot = null;
        if(target != null){
             targetRot =
                    drivetrain.getPose()
                            .minus(target.get()).getTranslation().getAngle()
                            .rotateBy(isFlipped ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0));
            drivetrain.setTargetPose(new Pose2d(drivetrain.getPose().getTranslation(), targetRot));
            Logger.recordOutput("Targeting/rotError", targetRot.minus(drivetrain.getRotation()));
            Logger.recordOutput("Targeting/target", target.get());
        }


        double xPercent   = -controller.getLeftY() * (Constants.IS_COMPETITION_ROBOT ? 1 : -1);


        Rotation2d linearDirection = drivetrain.getRotation().rotateBy(Rotation2d.fromDegrees(angleDegrees.get()));

        Logger.recordOutput("Targeting/targetDirection", new Pose2d(
                drivetrain.getPose().getTranslation(),
                linearDirection
        ));

        double rotPercent = target == null
                ? JoystickUtils.smartDeadzone(-controller.getRightX(), deadzone) * (Constants.IS_COMPETITION_ROBOT ? 1 : -1)
                : rotController.calculate(drivetrain.getRotation().getRadians(), targetRot.getRadians());;
        drivetrain.drivePercent((xPercent)*(linearDirection.getCos()>0?1:-1), linearDirection, rotPercent, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
