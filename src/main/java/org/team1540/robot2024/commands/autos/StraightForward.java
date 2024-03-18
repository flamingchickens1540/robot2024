package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class StraightForward extends AutoCommand {
    public StraightForward(Drivetrain drivetrain){
        super("StraightForward");

        addPath(
                PathHelper.fromChoreoPath("StraightForward.1", true, true),
                PathHelper.fromChoreoPath("StraightForward.2", true, true),
                PathHelper.fromChoreoPath("StraightForward.3"),
                PathHelper.fromChoreoPath("StraightForward.4"),
                PathHelper.fromChoreoPath("StraightForward.5"),
                PathHelper.fromChoreoPath("StraightForward.6")
        );

        addCommands(
//                getPath(0).getCommand(drivetrain)
//                Commands.waitSeconds(35),
                getPath(1).getCommand(drivetrain)
//                getPath(2).getCommand(drivetrain),
//                getPath(3).getCommand(drivetrain),
//                getPath(4).getCommand(drivetrain),
//                getPath(5).getCommand(drivetrain),
//                Commands.runOnce(drivetrain::stop)
        );
    }
}
