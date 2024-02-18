package org.team1540.robot2024;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.team1540.robot2024.Constants.Elevator.ElevatorState;
import org.team1540.robot2024.commands.FeedForwardCharacterization;
import org.team1540.robot2024.commands.DriveWithSpeakerTargetingCommand;
import org.team1540.robot2024.commands.SwerveDriveCommand;
import org.team1540.robot2024.commands.TrampShoot;
import org.team1540.robot2024.commands.elevator.ElevatorManualCommand;
import org.team1540.robot2024.commands.elevator.ElevatorSetpointCommand;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.indexer.StageTrampCommand;
import org.team1540.robot2024.commands.shooter.PrepareShooterCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.commands.autos.*;
import org.team1540.robot2024.commands.tramp.TrampStageSequence;
import org.team1540.robot2024.subsystems.drive.*;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.led.Leds;
import org.team1540.robot2024.subsystems.led.patterns.LedPatternFlame;
import org.team1540.robot2024.subsystems.shooter.*;
import org.team1540.robot2024.subsystems.tramp.Tramp;
import org.team1540.robot2024.subsystems.vision.AprilTagVision;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.AutoManager;
import org.team1540.robot2024.util.PhoenixTimeSyncSignalRefresher;
import org.team1540.robot2024.util.vision.VisionPoseAcceptor;

import static org.team1540.robot2024.Constants.SwerveConfig;
import static org.team1540.robot2024.Constants.isTuningMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
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
                // TODO: 2/16/2024 switch these back to the correct implementations when merging
//                drivetrain = Drivetrain.createReal(odometrySignalRefresher);
//                tramp = Tramp.createReal();
//                shooter = Shooter.createReal();
//                elevator = Elevator.createReal();
//                indexer = Indexer.createReal();
//                aprilTagVision = AprilTagVision.createReal(
//                        drivetrain::addVisionMeasurement,
//                        elevator::getPosition,
//                        new VisionPoseAcceptor(drivetrain::getChassisSpeeds, () -> 0.0));
//                drivetrain = Drivetrain.createReal(odometrySignalRefresher);
                drivetrain = Drivetrain.createReal(odometrySignalRefresher);
                tramp = Tramp.createReal();
                shooter = Shooter.createDummy();
                elevator = Elevator.createReal();
                indexer = Indexer.createReal();
                aprilTagVision = AprilTagVision.createDummy();
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drivetrain = Drivetrain.createSim();
                tramp = Tramp.createSim();
                shooter = Shooter.createSim();
                elevator = Elevator.createSim();
                indexer = Indexer.createSim();
                aprilTagVision = AprilTagVision.createSim(
                        drivetrain::addVisionMeasurement,
                        drivetrain::getPose,
                        elevator::getPosition,
                        new VisionPoseAcceptor(drivetrain::getChassisSpeeds, () -> 0.0));
                break;
            default:
                // Replayed robot, disable IO implementations
                drivetrain = Drivetrain.createDummy();
                tramp = Tramp.createDummy();
                shooter = Shooter.createDummy();
                elevator = Elevator.createDummy();
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
        leds.setFatalPattern(LedPatternFlame::new);
        new Trigger(DriverStation::isDSAttached)
                .onTrue(Commands.runOnce(leds::clearFatalPattern)
                            .ignoringDisable(true))
                .onFalse(Commands.runOnce(() -> leds.setFatalPattern(LedPatternFlame::new))
                            .ignoringDisable(true));
    }
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driver));
        elevator.setDefaultCommand(new ElevatorManualCommand(elevator, copilot));
//        indexer.setDefaultCommand(new IntakeCommand(indexer, tramp));

        driver.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
//        driver.y().toggleOnTrue(new DriveWithSpeakerTargetingCommand(drivetrain, driver));
        driver.y().onTrue(
                Commands.runOnce(
                        () -> drivetrain.setPose(new Pose2d(drivetrain.getPose().getTranslation(), new Rotation2d())),
                        drivetrain
                ).ignoringDisable(true)
        );


        copilot.rightBumper().whileTrue(new IntakeCommand(indexer, tramp));
//        copilot.leftBumper().onTrue(new ElevatorSetpointCommand(elevator, ElevatorState.BOTTOM));
//        copilot.a().onTrue(new ShootSequence(shooter, indexer))
//                .onFalse(Commands.runOnce(shooter::stopFlywheels, shooter));
        copilot.x().whileTrue(new ShootSequence(shooter, indexer));
        copilot.a().whileTrue(
                new ElevatorSetpointCommand(elevator, ElevatorState.BOTTOM).andThen(
                        new StageTrampCommand(tramp, indexer)
                ).andThen(
                        new ElevatorSetpointCommand(elevator, ElevatorState.AMP)
                )
        );
        copilot.b().whileTrue(new PrepareShooterCommand(shooter));
//        copilot.rightTrigger(0.5).whileTrue(new ElevatorSetpointCommand(elevator, ElevatorState.BOTTOM));
//        copilot.leftTrigger(0.5).whileTrue(new ElevatorSetpointCommand(elevator, ElevatorState.CLIMB));
        copilot.leftBumper().whileTrue(
                new TrampStageSequence(indexer, tramp, elevator).onlyIf(() -> !tramp.isNoteStaged()).andThen(new TrampShoot(tramp))
        ).onFalse(new ElevatorSetpointCommand(elevator, ElevatorState.BOTTOM));


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
        AutoManager.getInstance().addAuto(new PathVisualising(drivetrain));
        AutoManager.getInstance().addAuto(new DriveSinglePath("AmpLaneTaxi", drivetrain));
        AutoManager.getInstance().addAuto(new DriveSinglePath("AmpLaneSprint", drivetrain));
        AutoManager.getInstance().addDefaultAuto(new DriveSinglePath("CenterLaneTaxi", drivetrain));
        AutoManager.getInstance().addAuto(new DriveSinglePath("CenterLaneSprint", drivetrain));
        AutoManager.getInstance().addAuto(new DriveSinglePath("SourceLaneTaxi", drivetrain));
        AutoManager.getInstance().addAuto(new DriveSinglePath("SourceLaneSprint", drivetrain));
        AutoManager.getInstance().addAuto(new AutoCommand("Dwayne :skull:"));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
//        return null;
        return new FeedForwardCharacterization(elevator, elevator::setVoltage, elevator::getVelocity);
    }
}
