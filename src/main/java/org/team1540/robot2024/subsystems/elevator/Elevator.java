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
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final AverageFilter positionFilter = new AverageFilter(10);
    private double setpointMeters;


    public Elevator(ElevatorIO elevatorIO) {
        this.io = elevatorIO;
    }

    // periodic
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        MechanismVisualiser.setElevatorPosition(inputs.positionMeters);

        positionFilter.add(inputs.positionMeters);
    }
    
    public void setElevatorPosition(double positionMeters) {
        positionMeters = MathUtil.clamp(positionMeters, Constants.Elevator.ELEVATOR_MINIMUM_HEIGHT, Constants.Elevator.ELEVATOR_MAX_HEIGHT);
        setpointMeters = positionMeters;
        io.setSetpointMeters(setpointMeters);

        positionFilter.clear();
    }

    public boolean isAtSetpoint() {
        return MathUtil.isNear(setpointMeters, positionFilter.getAverage(), ERROR_TOLERANCE);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void stop() {
        io.setVoltage(0.0);
    }

    @AutoLogOutput
    public double getSetpoint() {
        return setpointMeters;
    }


}
