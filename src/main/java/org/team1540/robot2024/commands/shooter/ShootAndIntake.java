package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;

public class ShootAndIntake extends ParallelDeadlineGroup{
    public ShootAndIntake(Indexer indexer, Shooter shooter, double intakePercent, double feederSpeed, Rotation2d pivotAngle, double leftSetpoint, double rightSetpoint){
        super(
                Commands.sequence(
                        Commands.waitUntil(indexer::isNoteStaged),
                        Commands.waitUntil(() -> !indexer.isNoteStaged()),
                        Commands.waitSeconds(0.3)

                ),
                Commands.startEnd(
                        () -> {shooter.setPivotPosition(pivotAngle); shooter.setFlywheelSpeeds(leftSetpoint, rightSetpoint);},
                        () -> {shooter.setFlywheelSpeeds(0,0);}
                ),
                indexer.setIntakeAndFeeder(intakePercent, feederSpeed)
        );
        addRequirements(shooter, indexer);
    }
}
