package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLanePSubSprint extends AutoCommand {


    public CenterLanePSubSprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("CenterLanPSubSprint");
        addPath(
                PathHelper.fromChoreoPath("CenterLaneSprint", true, true)
        );

        addCommands(
                new ShootSequence(shooter, indexer),
                getPath(0).getCommand(drivetrain)
        );
    }
}
