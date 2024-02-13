package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.util.AutoCommand;
import org.team1540.robot2024.util.PathHelper;

public class SourceLanePHGFSprint extends AutoCommand {
    public SourceLanePHGFSprint(Drivetrain drivetrain){
        super("SourceLanePHGFSprint");

        addPath(
                PathHelper.fromChoreoPath("SourceLanePHGFSprint.1", true, true),
                PathHelper.fromChoreoPath("SourceLanePHGFSprint.2"),
                PathHelper.fromChoreoPath("SourceLanePHGFSprint.3"),
                PathHelper.fromChoreoPath("SourceLanePHGFSprint.4")
        );

        addCommands(
                getPath(0).getCommand(drivetrain),
                new PrintCommand("Past 1"),
                getPath(1).getCommand(drivetrain),
                new PrintCommand("Past 2"),
                getPath(2).getCommand(drivetrain),
                new PrintCommand("Past 3"),
                getPath(3).getCommand(drivetrain),
                new PrintCommand("Past 4")
        );
    }
}
