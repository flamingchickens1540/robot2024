package org.team1540.robot2024;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.team1540.robot2024.commands.FeedForwardCharacterization;
import org.team1540.robot2024.commands.climb.TrapAndClimbSequence;
import org.team1540.robot2024.commands.drivetrain.SwerveDriveCommand;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.indexer.StageTrampCommand;
import org.team1540.robot2024.commands.shooter.*;
import org.team1540.robot2024.commands.tramp.TrampScoreSequence;
import org.team1540.robot2024.commands.elevator.ElevatorManualCommand;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.autos.*;
import org.team1540.robot2024.commands.tramp.TrampStageSequence;
import org.team1540.robot2024.subsystems.drive.*;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.led.Leds;
import org.team1540.robot2024.subsystems.led.patterns.*;
import org.team1540.robot2024.subsystems.shooter.*;
import org.team1540.robot2024.subsystems.tramp.Tramp;
import org.team1540.robot2024.subsystems.vision.AprilTagVision;
import org.team1540.robot2024.util.CommandUtils;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.AutoManager;
import org.team1540.robot2024.util.PhoenixTimeSyncSignalRefresher;
import org.team1540.robot2024.util.shooter.ShooterSetpoint;
import org.team1540.robot2024.util.vision.AprilTagsCrescendo;


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
        AutoManager.getInstance().add(
                new AutoCommand(
                        "Drive FF Characterization",
                        new FeedForwardCharacterization(
                                drivetrain, drivetrain::runCharacterizationVolts, drivetrain::getCharacterizationVelocity
                        )
                )
        );

        AutoManager.getInstance().add(
                new AutoCommand(
                        "Flywheels FF Characterization",
                        new FeedForwardCharacterization(
                                shooter, volts -> shooter.setFlywheelVolts(volts, volts), () -> shooter.getLeftFlywheelSpeed() / 60
                        )
                )
        );

        AutoManager.getInstance().addDefault(new AmpLanePABCSprint(drivetrain, shooter, indexer));
        AutoManager.getInstance().add(new SourceLanePHGFSprint(drivetrain));

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
        ManualPivotCommand manualPivotCommand = new ManualPivotCommand(shooter, copilot);
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driver));
        elevator.setDefaultCommand(new ElevatorManualCommand(elevator, copilot));
        shooter.setDefaultCommand(manualPivotCommand);

        driver.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
//        driver.y().toggleOnTrue(new DriveWithSpeakerTargetingCommand(drivetrain, driver));
        driver.y().onTrue(Commands.runOnce(() -> {
            drivetrain.zeroFieldOrientationManual();
            drivetrain.setBrakeMode(true);
        }).ignoringDisable(true));
        Command targetDrive = new AutoShootPrepare(driver.getHID(), drivetrain, shooter);
        driver.a().toggleOnTrue(targetDrive);
        driver.rightStick().onTrue(Commands.runOnce(targetDrive::cancel));


        copilot.leftBumper().whileTrue(new TrampScoreSequence(tramp, indexer, elevator));
        Command intakeCommand = new IntakeCommand(indexer, tramp::isNoteStaged, 1);
        copilot.rightBumper().whileTrue(intakeCommand);

        copilot.povDown().whileTrue(indexer.commandRunIntake(-1));
        copilot.povUp().whileTrue(new IntakeCommand(indexer, tramp::isNoteStaged, 1, false));
        copilot.povRight().whileTrue(Commands.startEnd(() -> tramp.setPercent(1), tramp::stop, tramp));
//        copilot.povLeft().onTrue(CommandUtils.startStopTimed(
//                () -> leds.setPatternAll(() -> new LedPatternWave(DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red ? 0: 216), Leds.PatternCriticality.HIGH),
//                () -> leds.setPatternAll(() -> new LedPatternWave(DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red ? 0: 216), Leds.PatternCriticality.HIGH),
//                5
//        ));
        copilot.povLeft().whileTrue(new ShootSequence(shooter, indexer, ()->{
            ShooterSetpoint setpoint = new ShooterSetpoint(
                    Rotation2d.fromRadians(
                            Math.atan2(Constants.Targeting.SPEAKER_CENTER_HEIGHT - Constants.Shooter.Pivot.PIVOT_HEIGHT, drivetrain.getPose().getTranslation().getDistance(
                                    AprilTagsCrescendo.getInstance().getTag(AprilTagsCrescendo.Tags.SPEAKER_CENTER).toPose2d().getTranslation()
                            ))).minus(Constants.Shooter.Pivot.REAL_ZEROED_ANGLE),
                    8000, 6000);
            return setpoint;
        }));

        copilot.rightTrigger(0.95).whileTrue(Commands.startEnd(() -> tramp.setPercent(1), tramp::stop, tramp));
        copilot.leftTrigger(0.95).whileTrue(new TrapAndClimbSequence(drivetrain, elevator, null, tramp, indexer, shooter));


        copilot.x().whileTrue(new ShootSequence(shooter, indexer));
        copilot.a().whileTrue(new TrampStageSequence(indexer, tramp, elevator));
//        copilot.b().whileTrue(new ShootSequence(shooter, indexer, PODIUM_SHOOT));
//        copilot.b().whileTrue(new TuneShooterCommand(shooter, indexer));
        copilot.b().whileTrue(new IntakeAndFeed(indexer, () -> 1, () -> 0.5));
        copilot.y().whileTrue(new StageTrampCommand(tramp, indexer));

//        copilot.leftTrigger(0.5).whileTrue(new ElevatorSetpointCommand(elevator, ElevatorState.CLIMB));


        new Trigger(indexer::isNoteStaged).debounce(0.1)
                .onTrue(CommandUtils.rumbleCommand(driver.getHID(), 0.8, 1))
                .whileTrue(Commands.startEnd(() -> leds.setPattern(Leds.Zone.ELEVATOR_BACK, new LedPatternWave(0), Leds.PatternCriticality.EXTREME), () -> leds.clearPattern(Leds.Zone.ELEVATOR_BACK, Leds.PatternCriticality.EXTREME)));

        new Trigger(indexer::isNoteStaged).and(intakeCommand::isScheduled).onTrue(CommandUtils.rumbleCommand(driver.getHID(), 0.3, 0.4));

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
        AutoManager autos = AutoManager.getInstance();
        // Set up FF characterization routines
        if(isTuningMode()){
            AutoManager.getInstance().add(
                    new AutoCommand(
                            "Drive FF Characterization",
                            new FeedForwardCharacterization(
                                    drivetrain, drivetrain::runCharacterizationVolts, drivetrain::getCharacterizationVelocity
                            )
                    )
            );

            AutoManager.getInstance().add(
                    new AutoCommand(
                            "Flywheels FF Characterization",
                            new FeedForwardCharacterization(
                                    shooter, volts -> shooter.setFlywheelVolts(volts, volts), () -> shooter.getLeftFlywheelSpeed() / 60
                            )
                    )
            );
        }
        autos.addDefault(new AutoCommand("Dwayne :skull:"));
        autos.add(new AutoCommand("SubwooferShot", new ShootSequence(shooter, indexer)));
        autos.add(new DriveSinglePath("AmpLaneTaxi", drivetrain));
        autos.add(new DriveSinglePath("AmpLaneSprint", drivetrain));
        autos.add(new AmpLanePSprint(drivetrain, shooter, indexer));
        autos.add(new AmpLanePABCSprint(drivetrain, shooter, indexer));
        autos.add(new AmpLanePADESprint(drivetrain, shooter, indexer));
        autos.add(new DriveSinglePath("CenterLaneTaxi", drivetrain));
        autos.add(new DriveSinglePath("CenterLaneSprint", drivetrain, true, true));
        autos.add(new CenterLanePSubSprint(drivetrain, shooter, indexer));
        autos.add(new CenterLanePCBAD(drivetrain, shooter, indexer));
        autos.add(new CenterLanePCBAF(drivetrain, shooter, indexer));
        autos.add(new CenterLanePCBFSprint(drivetrain, shooter, indexer));
        autos.add(new CenterLanePBDA(drivetrain, shooter, indexer));
        autos.add(new CenterLanePSubCSubBSubASubFSub(drivetrain, shooter, indexer));
        autos.add(new CenterLanePSubCSubBSubFSub(drivetrain, shooter, indexer));
        autos.add(new DriveSinglePath("SourceLaneTaxi", drivetrain));
        autos.add(new DriveSinglePath("SourceLaneSprint", drivetrain));
        autos.add(new SourceLanePHGSprint(drivetrain, shooter, indexer));






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
