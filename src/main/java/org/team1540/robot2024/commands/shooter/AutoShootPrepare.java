package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.commands.drivetrain.DriveWithSpeakerTargetingCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.shooter.ShooterSetpoint;
import org.team1540.robot2024.util.vision.AprilTagsCrescendo;

public class AutoShootPrepare extends ParallelCommandGroup {
    public AutoShootPrepare(XboxController controller, Drivetrain drivetrain, Shooter shooter) {
        addCommands(
                new DriveWithSpeakerTargetingCommand(drivetrain,controller),
                new AutoShooterPrepare(drivetrain, shooter)
        );
    }
}
