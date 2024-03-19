package org.team1540.robot2024.subsystems.tramp;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.team1540.robot2024.Constants;

import static org.team1540.robot2024.Constants.Tramp.GEAR_RATIO;

public class TrampIOSim implements TrampIO {
    private final DCMotorSim sim = new DCMotorSim(DCMotor.getNEO(1), GEAR_RATIO, 0.025); //TODO: Fix MOI its probably wrong :D

    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(TrampIOInputs inputs) {
        sim.update(Constants.LOOP_PERIOD_SECS);
        inputs.velocityRPS = sim.getAngularVelocityRPM();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12, 12);
        sim.setInputVoltage(appliedVolts);
    }
}
