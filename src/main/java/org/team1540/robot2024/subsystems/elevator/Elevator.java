package org.team1540.robot2024.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private double setpointRots;


    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    // periodic
    public void periodic(){
        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator", elevatorInputs);

    }
    // do positional control stuff here
    
    public void goToSetpoint(double setpointMeters) {
        this.setpointRots = setpointMeters / Constants.Elevator.SOCKET_DIAMETER;
        elevatorIO.setSetpoint(setpointRots);
    }

    public boolean isAtSetpoint() {
        return elevatorInputs.positionMeters == setpointRots;
    }

    public void stop() {
        elevatorIO.stop();
    }
}
