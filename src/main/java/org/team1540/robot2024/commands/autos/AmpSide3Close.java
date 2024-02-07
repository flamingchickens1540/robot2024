package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.AutoCommand;
import org.team1540.robot2024.util.AutoManager;
import org.team1540.robot2024.util.PathPlannerHelper;

public class AmpSide3Close extends AutoCommand {

    public AmpSide3Close(Drivetrain drivetrain, Shooter shooter, Indexer indexer){
        super("AmpSide3Close");

        addPath(
                new PathPlannerHelper(drivetrain, "3CloseTop.1", true, true, true),
                new PathPlannerHelper(drivetrain, "3CloseTop.2", true),
                new PathPlannerHelper(drivetrain, "3CloseTop.3", true)
                );
        addCommands(
//                new ShootSequence(shooter, indexer),
                new PrintCommand("Starting command"),
                getPath(0).getCommand(),
                new PrintCommand("Past path 1"),
//                new ShootSequence(shooter, indexer),
                getPath(1).getCommand(),
//                new ShootSequence(shooter, indexer),
                getPath(2).getCommand()
//                new ShootSequence(shooter, indexer)
        );
    }
}
