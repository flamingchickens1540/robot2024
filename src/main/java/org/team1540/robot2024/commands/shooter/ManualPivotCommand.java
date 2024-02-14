package org.team1540.robot2024.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2024.subsystems.shooter.Shooter;

public class ManualShooterCommand extends Command{
        private final Shooter shooter;
        private final CommandXboxController controller;

        public ManualShooterCommand(Shooter shooter, CommandXboxController controller) {
            this.shooter = shooter;
            addRequirements(shooter);
            this.controller = controller;
        }
        @Override
        public void execute() {
            Rotation2d angleRotation2d = shooter.getPivotPosition();
            shooter.setPivotPosition(angleRotation2d.plus(Rotation2d.fromRotations(0.01*(controller.getLeftY()))));
            // controller values range from -1 to 1
            // pivot position will be within 90 degrees
        }

        @Override
        public boolean isFinished() {
            return shooter.isPivotAtSetpoint();
        }
    }

