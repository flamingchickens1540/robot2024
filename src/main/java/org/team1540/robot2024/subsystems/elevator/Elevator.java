package org.team1540.robot2024.subsystems.elevator;

import org.team1540.robot2024.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final TalonFX talon1 = new TalonFX(Constants.Elevator.talonId1);
    private final TalonFX talon2 = new TalonFX(Constants.Elevator.talonId2);
    private final ElevatorIO inputs;


    public Elevator(ElevatorIO inputs) {
        this.inputs = inputs;
    }

    // do positional control stuff here
}
