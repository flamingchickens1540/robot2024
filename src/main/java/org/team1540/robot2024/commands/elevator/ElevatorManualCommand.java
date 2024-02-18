package org.team1540.robot2024.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2024.subsystems.elevator.Elevator;

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
//        elevator.setElevatorPosition(
//                elevator.getSetpoint()
//                + Constants.Elevator.CRUISE_VELOCITY_MPS
//                * Constants.LOOP_PERIOD_SECS
//                * (copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis()));
//        elevator.setVoltage((copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis()) * 12 * 0.125);
        elevator.setVoltage((copilot.getRightY()) * 12 * 0.125);

    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
