package org.team1540.robot2024.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.subsystems.elevator.Elevator;

public class ElevatorManualCommand extends Command {
    private static final double DEADZONE = 0.3;
    private final Elevator elevator;
    private final CommandXboxController copilot;

    public ElevatorManualCommand(Elevator elevator, CommandXboxController copilot) {
        this.elevator = elevator;
        this.copilot = copilot;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double val = copilot.getRightY();
        if (Math.abs(val) > DEADZONE) {
            if (copilot.getRightY() < 0) {
                val += DEADZONE;
            } else {
                val -= DEADZONE;
            }
            elevator.setVoltage(-val * 12 * 0.18);
        } else {
            elevator.holdPosition();
        }


    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
