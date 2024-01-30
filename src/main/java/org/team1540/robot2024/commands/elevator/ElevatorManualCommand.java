package org.team1540.robot2024.commands.elevator;

import org.team1540.robot2024.Constants;
import org.team1540.robot2024.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ElevatorManualCommand extends Command {

    private final Elevator elevator;
    private final CommandXboxController copilot;

    public ElevatorManualCommand(Elevator elevator, CommandXboxController copilot) {
        this.elevator = elevator;
        this.copilot = copilot;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setVoltage(copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis() + Constants.Elevator.KG);
    }
}
