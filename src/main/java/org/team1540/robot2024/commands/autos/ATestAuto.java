package org.team1540.robot2024.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2024.commands.shooter.AutoShootPrepare;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class ATestAuto extends AutoCommand {

    public ATestAuto(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("!ATestAuto");
        addPath(
                PathHelper.fromChoreoPath("CenterLaneSprint.1", true, true),
                PathHelper.fromChoreoPath("CenterLaneSprint.1", false, true)
        );

        addCommands(
                ShootSequence.forAutoSubwoofer(shooter, indexer),
                Commands.parallel(
                        new AutoShootPrepare(drivetrain, shooter),
                        Commands.sequence(
                                drivetrain.commandCopyVisionPose(),
//                                Commands.deadline(
                                Commands.race(
                                        getPath(0).getCommand(drivetrain),
                                        Commands.sequence(
                                                Commands.waitSeconds(1),
                                                Commands.either(
//                                                        Commands.waitSeconds(0),
                                                        Commands.runOnce(()->{}),
                                                        Commands.waitSeconds(100),
                                                        ()->false
                                                )
                                        )
                                ),
//                                Commands.print("YO YO DIGGITY DAWG"),
//                                        Commands.runOnce(drivetrain::stop)
//                                        Commands.sequence(
//                                                Commands.waitSeconds(1)
//                                                Commands.runOnce(()->CommandScheduler.getInstance().cancelAll())
//                                        )
//                                ),
//                                Commands.runOnce(drivetrain::stop)
                                drivetrain.commandStop(),
                                getPath(1).getCommand(drivetrain,true)

                        )
                )
        );

        addRequirements(drivetrain, shooter, indexer);
    }
}
