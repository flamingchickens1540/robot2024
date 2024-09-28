package org.team1540.robot2024.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.util.LoggedTunableNumber;
import org.team1540.robot2024.util.math.JoystickUtils;

import java.util.function.Supplier;

import static org.team1540.robot2024.Constants.Targeting.*;

public class DriveWithCorrectionCommand2 extends Command {
    private final Drivetrain drivetrain;
    private final CommandXboxController controller;

    private final Supplier<Double> angleDegrees;
    private final Supplier<Pose2d> target;
    private static final double deadzone = 0.03;

    private boolean isFlipped;


    private final LoggedTunableNumber kP = new LoggedTunableNumber("Targeting/COR_KP", 0.9);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Targeting/COR_KI", 0);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Targeting/COR_KD", 0);
    private final PIDController rotController = new PIDController(ROT_KP, ROT_KI, ROT_KD);
    private final PIDController correctionController = new PIDController(kP.get(), kI.get(), kD.get());


    public DriveWithCorrectionCommand2(Drivetrain drivetrain, CommandXboxController controller, Supplier<Double> angleDegrees, Supplier<Pose2d> target) {

        this.drivetrain = drivetrain;
        this.controller = controller;
        this.angleDegrees = angleDegrees;
        this.target = target;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrain);
    }

    public DriveWithCorrectionCommand2(Drivetrain drivetrain, CommandXboxController controller, Supplier<Double> angleDegrees) {
        this(drivetrain, controller, angleDegrees, null);
    }

    @Override
    public void initialize() {
        rotController.reset();
        isFlipped = Constants.Targeting.getFlipped();
    }

    @Override
    public void execute() {
//        if(kP.hasChanged(hashCode())){
//            correctionController.setD(kP.get());
//        }
//        if(kI.hasChanged(hashCode())){
//            correctionController.setD(kI.get());
//        }
//        if(kD.hasChanged(hashCode())){
//            correctionController.setD(kD.get());
//        }



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
        double yPercent   = -controller.getLeftX() * (Constants.IS_COMPETITION_ROBOT ? 1 : -1);
        double linearMagnitude = JoystickUtils.smartDeadzone(Math.hypot(xPercent, yPercent), deadzone);
        Rotation2d linearDirection = new Rotation2d(xPercent, yPercent);

//        if(drivetrain.getRotation().rotateBy(Rotation2d.fromDegrees(angleDegrees.get())).getCos() * xPercent > 0){
            linearDirection = linearDirection.minus(Rotation2d.fromDegrees(correctionController.calculate(angleDegrees.get(), 0)));
//        }

        System.out.println("I am correctiong 1");
        Logger.recordOutput("Targeting/targetDirection", new Pose2d(
                drivetrain.getPose().getTranslation(),
                linearDirection.rotateBy(Rotation2d.fromDegrees(180))
        ));
        Logger.recordOutput("Targeting/angle", angleDegrees.get());
        Logger.recordOutput("Targeting/target", Constants.Targeting.getSpeakerPose());

        System.out.println();

        double rotPercent = target == null
                ? JoystickUtils.smartDeadzone(-controller.getRightX(), deadzone) * (Constants.IS_COMPETITION_ROBOT ? 1 : -1)
                : rotController.calculate(drivetrain.getRotation().getRadians(), targetRot.getRadians());;
        drivetrain.drivePercent(linearMagnitude, linearDirection, rotPercent, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
