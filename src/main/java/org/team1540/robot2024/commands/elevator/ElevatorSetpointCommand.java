package org.team1540.robot2024.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.Constants.Elevator.ElevatorState;
import org.team1540.robot2024.subsystems.elevator.Elevator;

public class ElevatorSetpointCommand extends Command {
    private final Elevator elevator;
    private final ElevatorState state;

    public ElevatorSetpointCommand(Elevator elevator, ElevatorState state) {
        this.elevator = elevator;
        this.state = state;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setElevatorPosition(state.heightMeters);
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtSetpoint();
    }

}
