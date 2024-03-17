package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.commands.drivetrain.DriveWithTargetingCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.vision.AprilTagsCrescendo;

public class OverStageShootPrepareWithTargeting extends ParallelCommandGroup {
    public OverStageShootPrepareWithTargeting(XboxController controller, Drivetrain drivetrain, Shooter shooter) {
        addCommands(
                new DriveWithTargetingCommand(drivetrain,controller, ()-> AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.SPEAKER_CENTER).toPose2d().plus(new Transform2d(0,-2,new Rotation2d()))),
                new OverStageShootPrepare(drivetrain, shooter)
        );
    }
}
