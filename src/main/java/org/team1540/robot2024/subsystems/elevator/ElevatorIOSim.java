package org.team1540.robot2024.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import static org.team1540.robot2024.Constants.Elevator.*;
import static org.team1540.robot2024.Constants.LOOP_PERIOD_SECS;

public class ElevatorIOSim implements ElevatorIO{
    // fields
    private final ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getFalcon500Foc(2), ELEVATOR_GEAR_RATIO, SIM_CARRIAGE_MASS_KG, SIM_DRUM_RADIUS_METERS, ELEVATOR_MINIMUM_HEIGHT, ELEVATOR_MAX_HEIGHT, true, ELEVATOR_MINIMUM_HEIGHT);
    private double elevatorAppliedVolts = 0.0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        elevatorSim.update(LOOP_PERIOD_SECS);

        inputs.positionMeters = elevatorSim.getPositionMeters();
        inputs.velocityRPM = elevatorSim.getVelocityMetersPerSecond();
        inputs.voltage = elevatorAppliedVolts;
        inputs.current = new double[]{elevatorSim.getCurrentDrawAmps()};
        inputs.upperLimit = elevatorSim.hasHitUpperLimit();
        inputs.lowerLimit = elevatorSim.hasHitLowerLimit();
    }

    public void setElevatorAppliedVolts(double volts){
        elevatorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0); //TODO: check this range
        elevatorSim.setInputVoltage(elevatorAppliedVolts);
    }
}
