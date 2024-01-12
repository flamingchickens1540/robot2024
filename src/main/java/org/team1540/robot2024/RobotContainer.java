package org.team1540.robot2024;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2024.commands.FeedForwardCharacterization;
import org.team1540.robot2024.commands.SwerveDriveCommand;
import org.team1540.robot2024.subsystems.drive.*;
import org.team1540.robot2024.util.swerve.SwerveFactory;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    public final Drivetrain drivetrain;

    // Controller
    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController copilot = new CommandXboxController(1);

    // Dashboard inputs
    public final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drivetrain =
                        new Drivetrain(
                                new GyroIONavx(),
                                new ModuleIOTalonFX(SwerveFactory.getModuleMotors(3, SwerveFactory.SwerveCorner.FRONT_LEFT)),
                                new ModuleIOTalonFX(SwerveFactory.getModuleMotors(4, SwerveFactory.SwerveCorner.FRONT_RIGHT)),
                                new ModuleIOTalonFX(SwerveFactory.getModuleMotors(7, SwerveFactory.SwerveCorner.BACK_LEFT)),
                                new ModuleIOTalonFX(SwerveFactory.getModuleMotors(1, SwerveFactory.SwerveCorner.BACK_RIGHT)));
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drivetrain =
                        new Drivetrain(
                                new GyroIO() {},
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new ModuleIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                drivetrain =
                        new Drivetrain(
                                new GyroIO() {
                                },
                                new ModuleIO() {
                                },
                                new ModuleIO() {
                                },
                                new ModuleIO() {
                                },
                                new ModuleIO() {
                                });
                break;
        }


        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up FF characterization routines
        autoChooser.addOption(
                "Drive FF Characterization",
                new FeedForwardCharacterization(
                        drivetrain, drivetrain::runCharacterizationVolts, drivetrain::getCharacterizationVelocity));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driver));
        driver.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        driver.b().onTrue(
                Commands.runOnce(
                        () -> drivetrain.setPose(new Pose2d(drivetrain.getPose().getTranslation(), new Rotation2d())),
                        drivetrain
                ).ignoringDisable(true)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
