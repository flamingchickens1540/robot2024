package org.team1540.robot2024.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.subsystems.drive.Drivetrain;

public class DriveWithAmpSideLock extends SequentialCommandGroup {
    public DriveWithAmpSideLock(Drivetrain drivetrain, XboxController controller){
        addCommands(
            new DriveWithTargetingCommand(drivetrain, controller, ()->drivetrain.getPose().plus(new Transform2d(0,DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Blue ? -1 : 1, new Rotation2d())))
        );
    }
}
