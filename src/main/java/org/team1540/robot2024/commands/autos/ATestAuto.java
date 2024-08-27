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
import org.team1540.robot2024.util.math.Triplet;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ATestAuto extends AutoCommand {

    public ATestAuto(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("!ATestAuto");
        addPath(
                PathHelper.fromChoreoPath("New Path.1", true, true),
                PathHelper.fromChoreoPath("New Path.2", false, true),
                PathHelper.fromChoreoPath("New Path.3", false, true),
                PathHelper.fromChoreoPath("New Path Switch 1.1", false, true),
                PathHelper.fromChoreoPath("New Path Switch 2.1", false, true)
        );
        addCommands(
//                getPath(0).getCommand(drivetrain),

                getPath(0).withInterrupt(
                        ()->Commands.sequence(
                                getPath(0).getCommand(drivetrain),
                                Commands.waitSeconds(1),
                                getPath(1).withInterrupt(
                                        ()->
                                        Commands.sequence(
                                                getPath(1).getCommand(drivetrain),
                                                Commands.waitSeconds(1),
                                                getPath(2).getCommand(drivetrain)
                                        ),
                                        ()->getPath(4).getCommand(drivetrain, false),
                                        new Triplet<>(0, ()->true, Commands::none),
                                        new Triplet<>(1, ()->false, Commands::none)
                                )
                        ),
                        ()->Commands.sequence(
                                getPath(3).getCommand(drivetrain, false),
                                Commands.waitSeconds(1),
                                getPath(2).getCommand(drivetrain)
                        ),
                        new Triplet<>(0, ()->true, Commands::none),
                        new Triplet<>(1, ()->false, Commands::none)
                ),
                Commands.print("HALLELUJAH")

                //,
//
//                getPath(0).withInterrupt(drivetrain, false,
//                        new Triplet<>(0, ()->false, getPath(3).getCommand(drivetrain, false)),
//                        new Triplet<>(1, ()->false, getPath(3).getCommand(drivetrain,false))
//                ),
//                Commands.waitSeconds(1),
//                getPath(1).getCommand(drivetrain),
//                getPath(1).withInterrupt(drivetrain, false,
//                        new Triplet<>(0, ()->false, getPath(4).getCommand(drivetrain, false)),
//                        new Triplet<>(1, ()->false, getPath(4).getCommand(drivetrain,false))
//                ),
//                Commands.waitSeconds(1),
//                getPath(2).getCommand(drivetrain)
        );
        addRequirements(drivetrain, shooter, indexer);



//        addPath(
//                PathHelper.fromChoreoPath("TestPath.1", true, true),
//                PathHelper.fromChoreoPath("TestPath.2", false, true),
//                PathHelper.fromChoreoPath("TestPath.3", false, true)
//        );
//        addCommands(
//                getPath(0).withInterrupt(drivetrain, false,
//                        new Triplet<>(0, ()->false, getPath(3).getCommand(drivetrain, false)),
//                        new Triplet<>(1, ()->false, getPath(3).getCommand(drivetrain,false))
//                ),
//
//        );
    }
}

