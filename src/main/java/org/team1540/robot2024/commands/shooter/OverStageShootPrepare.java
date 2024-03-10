package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.commands.drivetrain.DriveWithSpeakerTargetingCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.shooter.Shooter;

public class OverStageShootPrepare extends ParallelCommandGroup {
    public OverStageShootPrepare(XboxController controller, Drivetrain drivetrain, Shooter shooter) {
        addCommands(
                new DriveWithSpeakerTargetingCommand(drivetrain,controller),
                new OverStageShooterPrepare(drivetrain, shooter)
        );
    }
}
