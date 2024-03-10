package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class DriveSinglePath extends AutoCommand {
    public DriveSinglePath(String pathName, Drivetrain drivetrain){
        this(pathName, drivetrain, true, true);
    }
    public DriveSinglePath(String pathName, Drivetrain drivetrain, boolean shouldReset, boolean canFlip){
        super(pathName);

        addPath(
                PathHelper.fromChoreoPath(pathName, shouldReset, canFlip)
        );

        addCommands(
                getPath(0).getCommand(drivetrain),
                Commands.runOnce(drivetrain::stop)
        );
    }
}
