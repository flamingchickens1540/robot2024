package org.team1540.robot2024.commands.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.util.auto.PathHelper;
import org.team1540.robot2024.util.vision.AprilTagsCrescendo;

import java.util.function.Supplier;

import static org.team1540.robot2024.Constants.Auto.STAGE_PATH_CONSTRAINTS;

public class DriveWithChainAlignment extends SequentialCommandGroup {
    public DriveWithChainAlignment(Drivetrain drivetrain, XboxController controller){
//        new DriveWithTargetingCommand(drivetrain, controller, ()->drivetrain.getPose().plus(new Transform2d(0, DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Blue ? -1 : 1, new Rotation2d())));
//        new DriveWithTargetingCommand(drivetrain, controller, ()->drivetrain.getPose().plus(new Transform2d(
//                AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.AMP).getRotation().toRotation2d().getCos(),
//                AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.AMP).getRotation().toRotation2d().getSin(),
//                new Rotation2d()
//        )));
        addCommands(
                new DriveWithTargetingCommand(drivetrain, controller, ()->drivetrain.getPose().plus(aim(drivetrain::getPose)))
        );
    }


    private Transform2d aim(Supplier<Pose2d> position){
        double far = position.get().getTranslation().getDistance(
                AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.CLIMB_FAR).getTranslation().toTranslation2d());
        double amp = position.get().getTranslation().getDistance(
                AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.CLIMB_AMP).getTranslation().toTranslation2d());
        double source = position.get().getTranslation().getDistance(
                AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.CLIMB_SOURCE).getTranslation().toTranslation2d());
        Rotation2d targetAngle;
        if(far < amp && far < source){
             targetAngle = AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.CLIMB_FAR).getRotation().toRotation2d();
        }
        else if(amp < source){
            targetAngle = AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.CLIMB_AMP).getRotation().toRotation2d();
        }
        else{
            targetAngle = AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.CLIMB_SOURCE).getRotation().toRotation2d();
        }
        return new Transform2d(targetAngle.getCos(), targetAngle.getSin(), new Rotation2d());
    }
}


