package org.team1540.robot2024.commands.shooter;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;

import java.util.function.BooleanSupplier;

public class DefaultShooterCommand extends SequentialCommandGroup {

    private boolean hasNote = false;
    private boolean shouldShootPrepare = false;

    private BooleanSupplier shootPrepareSupplier;

    public DefaultShooterCommand(Drivetrain drivetrain, Shooter shooter, Indexer indexer){
        shootPrepareSupplier = ()->{
            Pose2d pose = DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Blue ? drivetrain.getPose() : GeometryUtil.flipFieldPose(drivetrain.getPose());
            return pose.getTranslation().getX() < Units.inchesToMeters(231.2);
        };
        addCommands(
                Commands.runOnce(()->shouldShootPrepare = shootPrepareSupplier.getAsBoolean()),
                Commands.runOnce(()->hasNote = indexer.isNoteStaged()),
                Commands.either(
                        Commands.either(
                                new LeadingShootPrepare(drivetrain, shooter, Constants.Shooter.Flywheels.LEFT_RPM*0.75, Constants.Shooter.Flywheels.RIGHT_RPM*0.75).alongWith(Commands.runOnce(()->hasNote = true)),
                                new LeadingShootPrepare(drivetrain, shooter, 0, 0).alongWith(Commands.runOnce(()->hasNote = true)),
                                indexer::isNoteStaged
                        ).alongWith(Commands.runOnce(()-> shouldShootPrepare = true)),
                        new OverStageShootPrepare(drivetrain::getPose, shooter, 0, 0).alongWith(Commands.runOnce(()-> shouldShootPrepare = false)),
                        shootPrepareSupplier
                ).until(()-> shootPrepareSupplier.getAsBoolean() != shouldShootPrepare || indexer.isNoteStaged() != hasNote).repeatedly()
        );
    }
}
