package org.team1540.robot2024.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2024.util.MechanismVisualiser;
import org.team1540.robot2024.util.math.AverageFilter;

import static org.team1540.robot2024.Constants.Elevator.*;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private final AverageFilter elevatorPositionFilter = new AverageFilter(10);
    private double setpointMeters;


    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
        setVoltage(10);
    }

    // periodic
    public void periodic(){
        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator", elevatorInputs);
        MechanismVisualiser.setElevatorPosition(elevatorInputs.positionMeters);

        elevatorPositionFilter.add(elevatorInputs.positionMeters);
    }
    
    public void goToSetpoint(double newSetpointMeters) {
        setpointMeters = newSetpointMeters;
        elevatorIO.setPositionMeters(setpointMeters);

        elevatorPositionFilter.clear();
    }

    public boolean isAtSetpoint() {
        return MathUtil.isNear(setpointMeters, elevatorPositionFilter.getAverage(), ERROR_TOLERANCE);
    }

    public void setVoltage(double voltage) {
        elevatorIO.setVoltage(voltage);
    }

    public void stop() {
        elevatorIO.setVoltage(0.0);
    }
}
