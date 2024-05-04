package org.team1540.robot2024.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.util.LoggedTunableNumber;

public class KidModeSwerveDriveCommand extends Command {
    private final Drivetrain drivetrain;
    private final CommandXboxController controller;
    private double maxSpeedPercentage;
    private double maxRotsPercentage;

    public KidModeSwerveDriveCommand(Drivetrain drivetrain, CommandXboxController controller){
        this.drivetrain = drivetrain;
        this.controller = controller;
    }

    public void execute() {
        //TODO: check deadbands bc of diff controllers
        double xPercent = MathUtil.applyDeadband((-controller.getLeftY()), 0.1);
        double yPercent = MathUtil.applyDeadband((-controller.getLeftX()), 0.1);
        double rotPercent = MathUtil.applyDeadband((-controller.getRightX()), 0.1);

        LoggedTunableNumber kidModeSpeedPercent = new LoggedTunableNumber("kidMode/maxSpeedPercentage", 0.3);
        LoggedTunableNumber kidModeRotsPercent = new LoggedTunableNumber("kidMode/maxRotsPercentage", 0.5);

        drivetrain.drivePercent(xPercent * kidModeSpeedPercent.get(), yPercent * kidModeSpeedPercent.get(), rotPercent * kidModeRotsPercent.get(), true);
    }


}
