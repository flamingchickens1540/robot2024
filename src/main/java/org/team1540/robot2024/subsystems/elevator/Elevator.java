package org.team1540.robot2024.subsystems.elevator;

import edu.wpi.first.math.MathUtil;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team1540.robot2024.Constants;
import org.team1540.robot2024.util.MechanismVisualiser;
import org.team1540.robot2024.util.math.AverageFilter;

import com.ctre.phoenix6.Utils;

import static org.team1540.robot2024.Constants.Elevator.*;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private final AverageFilter elevatorPositionFilter = new AverageFilter(10);
    private double setpointMeters;


    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    // periodic
    public void periodic(){
        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator", elevatorInputs);
        MechanismVisualiser.setElevatorPosition(elevatorInputs.positionMeters);

        elevatorPositionFilter.add(elevatorInputs.positionMeters);

        // if (elevatorInputs.lowerLimit) {
        //     setpointMeters = Constants.Elevator.ELEVATOR_MINIMUM_HEIGHT;
        // }
        // else if (elevatorInputs.upperLimit) {
        //     setpointMeters = Constants.Elevator.ELEVATOR_MAX_HEIGHT;
        // }
    }
    
    public void goToSetpoint(double newSetpointMeters) {
        newSetpointMeters = MathUtil.clamp(newSetpointMeters, Constants.Elevator.ELEVATOR_MINIMUM_HEIGHT, Constants.Elevator.ELEVATOR_MAX_HEIGHT);
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

    @AutoLogOutput
    public double getSetpoint() {
        return setpointMeters;
    }


}
