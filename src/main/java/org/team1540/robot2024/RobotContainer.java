package org.team1540.robot2024;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.team1540.robot2024.commands.FeedForwardCharacterization;
import org.team1540.robot2024.commands.climb.ClimbSequence;
import org.team1540.robot2024.commands.climb.ScoreInTrap;
import org.team1540.robot2024.commands.climb.TrapAndClimbSequence;
import org.team1540.robot2024.commands.drivetrain.DriveWithSpeakerTargetingCommand;
import org.team1540.robot2024.commands.drivetrain.SwerveDriveCommand;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.indexer.StageTrampCommand;
import org.team1540.robot2024.commands.shooter.ManualPivotCommand;
import org.team1540.robot2024.commands.tramp.TrampScoreSequence;
import org.team1540.robot2024.commands.elevator.ElevatorManualCommand;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.PrepareShooterCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.commands.autos.*;
import org.team1540.robot2024.commands.tramp.TrampShoot;
import org.team1540.robot2024.commands.tramp.TrampStageSequence;
import org.team1540.robot2024.subsystems.drive.*;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.led.Leds;
import org.team1540.robot2024.subsystems.led.patterns.LedPatternFlame;
import org.team1540.robot2024.subsystems.led.patterns.LedPatternRSLState;
import org.team1540.robot2024.subsystems.shooter.*;
import org.team1540.robot2024.subsystems.tramp.Tramp;
import org.team1540.robot2024.subsystems.vision.AprilTagVision;
import org.team1540.robot2024.util.CommandUtils;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.AutoManager;
import org.team1540.robot2024.util.PhoenixTimeSyncSignalRefresher;
import org.team1540.robot2024.util.vision.VisionPoseAcceptor;


import static org.team1540.robot2024.Constants.Shooter.Pivot.HUB_SHOOT;
import static org.team1540.robot2024.Constants.SwerveConfig;
import static org.team1540.robot2024.Constants.isTuningMode;

public class RobotContainer {
    // Subsystems
    public final Drivetrain drivetrain;
    public final Tramp tramp;
    public final Shooter shooter;
    public final Elevator elevator;
    public final Indexer indexer;
    public final AprilTagVision aprilTagVision;
    public final Leds leds = new Leds();

    // Controller
    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController copilot = new CommandXboxController(1);

    public final PhoenixTimeSyncSignalRefresher odometrySignalRefresher = new PhoenixTimeSyncSignalRefresher(SwerveConfig.CAN_BUS);

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                elevator = Elevator.createReal();
                drivetrain = Drivetrain.createReal(odometrySignalRefresher,elevator::getVelocity);
                tramp = Tramp.createReal();
                shooter = Shooter.createReal();
                indexer = Indexer.createReal();
                aprilTagVision = AprilTagVision.createReal(
                        drivetrain::addVisionMeasurement,
                        elevator::getPosition);
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                elevator = Elevator.createSim();
                drivetrain = Drivetrain.createSim(elevator::getVelocity);
                tramp = Tramp.createSim();
                shooter = Shooter.createSim();

                indexer = Indexer.createSim();
                aprilTagVision = AprilTagVision.createSim(
                        drivetrain::addVisionMeasurement,
                        drivetrain::getPose,
                        elevator::getPosition);
                break;
            default:
                // Replayed robot, disable IO implementations
                elevator = Elevator.createDummy();
                drivetrain = Drivetrain.createDummy();
                tramp = Tramp.createDummy();
                shooter = Shooter.createDummy();

                indexer = Indexer.createDummy();
                aprilTagVision = AprilTagVision.createDummy();
                break;
        }

        // Set up FF characterization routines
        AutoManager.getInstance().addAuto(
                new AutoCommand(
                        "Drive FF Characterization",
                        new FeedForwardCharacterization(
                                drivetrain, drivetrain::runCharacterizationVolts, drivetrain::getCharacterizationVelocity
                        )
                )
        );

        AutoManager.getInstance().addAuto(
                new AutoCommand(
                        "Flywheels FF Characterization",
                        new FeedForwardCharacterization(
                                shooter, volts -> shooter.setFlywheelVolts(volts, volts), () -> shooter.getLeftFlywheelSpeed() / 60
                        )
                )
        );

        AutoManager.getInstance().addDefaultAuto(new AmpLanePABCSprint(drivetrain, shooter, indexer));
        AutoManager.getInstance().addAuto(new SourceLanePHGFSprint(drivetrain));

        configureAutoRoutines();
        // Configure the button bindings
        configureButtonBindings();
        configureLedBindings();
    }

    private void configureLedBindings() {
//        Runnable onDisconnect = () -> leds.setPatternAll(LedPatternFlame::new, Leds.PatternCriticality.HIGH);
//        onDisconnect.run();
//        new Trigger(DriverStation::isDSAttached)
//                .onTrue(Commands.runOnce(() -> leds.clearPatternAll(Leds.PatternCriticality.HIGH))
//                            .ignoringDisable(true))
//                .onFalse(Commands.runOnce(onDisconnect).ignoringDisable(true));
    }

    private void configureButtonBindings() {
        Command rumbleDriver = CommandUtils.startStopTimed(
                () -> driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.3),
                () -> driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0),
                1);

        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driver));
        elevator.setDefaultCommand(new ElevatorManualCommand(elevator, copilot));
        shooter.setDefaultCommand(new ManualPivotCommand(shooter, copilot));

        driver.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
//        driver.y().toggleOnTrue(new DriveWithSpeakerTargetingCommand(drivetrain, driver));
        driver.y().onTrue(Commands.runOnce(() -> {
            drivetrain.zeroFieldOrientationManual();
            drivetrain.setBrakeMode(true);
        }).ignoringDisable(true));

//        driver.a().whileTrue(new DriveWithSpeakerTargetingCommand(drivetrain, driver));


        copilot.leftBumper().whileTrue(new TrampScoreSequence(tramp, indexer, elevator));
        copilot.rightBumper().whileTrue(new IntakeCommand(indexer, tramp::isNoteStaged, 1));

        copilot.povDown().whileTrue(indexer.commandRunIntake(-1));
        copilot.povUp().whileTrue(new ClimbSequence(drivetrain, elevator, null, tramp, indexer, shooter));
        copilot.povRight().whileTrue(Commands.startEnd(() -> tramp.setPercent(1), tramp::stop, tramp));

        copilot.rightTrigger(0.95).whileTrue(Commands.startEnd(() -> tramp.setPercent(1), tramp::stop, tramp));
        copilot.leftTrigger(0.95).whileTrue(new ClimbSequence(drivetrain, elevator, null, tramp, indexer, shooter));


        copilot.x().whileTrue(new ShootSequence(shooter, indexer, HUB_SHOOT));
        copilot.a().whileTrue(new TrampStageSequence(indexer, tramp, elevator));
        copilot.b().onTrue(new PrepareShooterCommand(shooter, HUB_SHOOT));
        copilot.y().whileTrue(new StageTrampCommand(tramp, indexer));

//        copilot.leftTrigger(0.5).whileTrue(new ElevatorSetpointCommand(elevator, ElevatorState.CLIMB));


        new Trigger(indexer::isNoteStaged).onTrue(rumbleDriver);

        new Trigger(RobotController::getUserButton).toggleOnTrue(Commands.startEnd(
                () -> {
                    elevator.setBrakeMode(false);
                    leds.setPatternAll(() -> new LedPatternRSLState(Color.kMagenta), Leds.PatternCriticality.EXTREME);
                    Commands.sequence(
                            Commands.waitSeconds(5),
                            Commands.runOnce(() ->leds.clearPatternAll(Leds.PatternCriticality.EXTREME))
                    ).schedule();
                },
                () -> {
                    elevator.setBrakeMode(true);
                    leds.clearPatternAll(Leds.PatternCriticality.EXTREME);
                }
        ));



    }

    private void configureAutoRoutines(){
        // Set up FF characterization routines
        if(isTuningMode()){
            AutoManager.getInstance().addAuto(
                    new AutoCommand(
                            "Drive FF Characterization",
                            new FeedForwardCharacterization(
                                    drivetrain, drivetrain::runCharacterizationVolts, drivetrain::getCharacterizationVelocity
                            )
                    )
            );

            AutoManager.getInstance().addAuto(
                    new AutoCommand(
                            "Flywheels FF Characterization",
                            new FeedForwardCharacterization(
                                    shooter, volts -> shooter.setFlywheelVolts(volts, volts), () -> shooter.getLeftFlywheelSpeed() / 60
                            )
                    )
            );
        }

        AutoManager.getInstance().addAuto(new AmpLanePABCSprint(drivetrain, shooter, indexer));
        AutoManager.getInstance().addAuto(new SourceLanePHGFSprint(drivetrain));
        AutoManager.getInstance().addAuto(new DriveSinglePath("AmpLaneTaxi", drivetrain));
        AutoManager.getInstance().addAuto(new DriveSinglePath("AmpLaneSprint", drivetrain));
        AutoManager.getInstance().addDefaultAuto(new DriveSinglePath("CenterLaneTaxi", drivetrain));
        AutoManager.getInstance().addAuto(new DriveSinglePath("CenterLaneSprint", drivetrain, true, true));
        AutoManager.getInstance().addAuto(new DriveSinglePath("SourceLaneTaxi", drivetrain));
        AutoManager.getInstance().addAuto(new DriveSinglePath("SourceLaneSprint", drivetrain));
        AutoManager.getInstance().addAuto(new DriveSinglePath("SQUARFE", drivetrain));
        AutoManager.getInstance().addAuto(new DriveSinglePath("SQUARFE (1)", drivetrain));
        AutoManager.getInstance().addAuto(new AutoCommand("Dwayne :skull:"));
        AutoManager.getInstance().addAuto(new AmpLanePADESprint(drivetrain, shooter, indexer));
        AutoManager.getInstance().addAuto(new CenterLanePCBFSprint(drivetrain, shooter, indexer));
        AutoManager.getInstance().addAuto(new SourceLanePHGSprint(drivetrain, shooter, indexer));
        AutoManager.getInstance().addAuto(new AmpLanePSprint(drivetrain, shooter, indexer));
        AutoManager.getInstance().addAuto(new CenterLaneSprintBonus(drivetrain));
        AutoManager.getInstance().addAuto(new StraightForward(drivetrain));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutoManager.getInstance().getSelected();
    }
}
