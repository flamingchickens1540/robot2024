package org.team1540.robot2024.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.util.LoggedTunableNumber;
import org.team1540.robot2024.util.math.JoystickUtils;

public class KidModeSwerveDriveCommand extends Command {
    private final Drivetrain drivetrain;
    private final CommandXboxController controller;
    private final LoggedTunableNumber kidModeSpeedPercent = new LoggedTunableNumber("kidMode/maxSpeedPercentage", 0.2);
    private final LoggedTunableNumber kidModeRotsPercent = new LoggedTunableNumber("kidMode/maxRotsPercentage", 0.4);
    private final LoggedTunableNumber deadbandLinear = new LoggedTunableNumber("kidMode/deadband/linear", 0.1);
    private final LoggedTunableNumber deadbandRotational = new LoggedTunableNumber("kidMode/deadband/rotational", 0.1);


    public KidModeSwerveDriveCommand(Drivetrain drivetrain, CommandXboxController controller){
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
    }

    public void execute() {
        double xPercent = -controller.getLeftY();
        double yPercent = -controller.getLeftX();
        double rotPercent = MathUtil.applyDeadband((-controller.getRightX()), deadbandRotational.get());

        double linearMagnitude = JoystickUtils.smartDeadzone(Math.hypot(xPercent, yPercent), deadbandLinear.get());
        Rotation2d linearDirection = new Rotation2d(xPercent, yPercent);

        drivetrain.drivePercent(linearMagnitude * kidModeSpeedPercent.get(), linearDirection, rotPercent * kidModeRotsPercent.get(), true);
    }


}
