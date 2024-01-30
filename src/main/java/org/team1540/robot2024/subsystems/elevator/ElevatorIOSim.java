package org.team1540.robot2024.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import static org.team1540.robot2024.Constants.Elevator.*;
import static org.team1540.robot2024.Constants.LOOP_PERIOD_SECS;

public class ElevatorIOSim implements ElevatorIO{
    // fields
    private final ElevatorSim elevatorSim =
            new ElevatorSim(
                DCMotor.getFalcon500Foc(2),
                    GEAR_RATIO, SIM_CARRIAGE_MASS_KG,
                    SIM_DRUM_RADIUS_METERS,ELEVATOR_MINIMUM_HEIGHT,
                    ELEVATOR_MAX_HEIGHT,
                    true,
                    ELEVATOR_MINIMUM_HEIGHT);
    private double elevatorAppliedVolts = 0.0;
    private final ProfiledPIDController controller =
            new ProfiledPIDController(
                    KP,
                    KI,
                    KD,
                    new TrapezoidProfile.Constraints(CRUISE_VELOCITY_MPS, MAXIMUM_ACCELERATION_MPS2));
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(KS, KG, KV);
    private boolean isClosedLoop;
    private TrapezoidProfile.State setpoint;

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        if (isClosedLoop) {
            elevatorAppliedVolts =
                    controller.calculate(elevatorSim.getPositionMeters(), setpoint)
                    + feedforward.calculate(
                            controller.getSetpoint().position,
                            controller.getSetpoint().velocity);
        }
        elevatorSim.setInputVoltage(elevatorAppliedVolts);
        elevatorSim.update(LOOP_PERIOD_SECS);

        inputs.positionMeters = elevatorSim.getPositionMeters();
        inputs.velocityRPM = elevatorSim.getVelocityMetersPerSecond();
        inputs.voltage = elevatorAppliedVolts;
        inputs.current = new double[]{elevatorSim.getCurrentDrawAmps()};
        inputs.upperLimit = elevatorSim.hasHitUpperLimit();
        inputs.lowerLimit = elevatorSim.hasHitLowerLimit();
    }

    @Override
    public void setVoltage(double volts) {
        isClosedLoop = false;
        elevatorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0); //TODO: check this range
    }
    @Override
    public void setPositionMeters(double position) {
        isClosedLoop = true;
        setpoint = new TrapezoidProfile.State(position, 0.0);
    }
}
