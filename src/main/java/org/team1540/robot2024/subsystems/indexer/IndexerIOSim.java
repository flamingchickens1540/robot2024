package org.team1540.robot2024.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import org.team1540.robot2024.Constants;

import static org.team1540.robot2024.Constants.Indexer.*;

public class IndexerIOSim implements IndexerIO {


    private final DCMotorSim intakeSim = new DCMotorSim(DCMotor.getNEO(1), INTAKE_GEAR_RATIO,0.025);
    private final DCMotorSim feederSim = new DCMotorSim(DCMotor.getNEO(1), FEEDER_GEAR_RATIO,0.025);
//    private final SimDeviceSim beamBreakSim = new SimDeviceSim("Indexer Beam Break");
    private final PIDController feederSimPID = new PIDController(FEEDER_KP, FEEDER_KI,FEEDER_KD);

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        intakeSim.update(Constants.LOOP_PERIOD_SECS);
        feederSim.update(Constants.LOOP_PERIOD_SECS);
        inputs.intakeCurrentAmps = intakeSim.getCurrentDrawAmps();
//        inputs.intakeVoltage = intakeSim.getBusVoltage() * intakeSim.getAppliedOutput();
        inputs.intakeVelocityRPM = intakeSim.getAngularVelocityRPM();
        inputs.feederCurrentAmps = feederSim.getCurrentDrawAmps();
//        inputs.feederVoltage = feederSim.getBusVoltage() * feederSim.getAppliedOutput();
        inputs.feederVelocityRPM = feederSim.getAngularVelocityRPM();
//        inputs.noteInIntake = beamBreakSim.getBoolean("Indexer Beam Break").get();
        inputs.setpoint = feederSimPID.getSetpoint();
        inputs.feederVelocityRadPerSec = feederSim.getAngularVelocityRadPerSec();
        inputs.feederPositionError = feederSimPID.getPositionError();

        // this is a very funny line of code, and absolutely does not belong here, but I don't know how to do this otherwise
        feederSim.setState(feederSim.getAngularPositionRad(), feederSim.getAngularVelocityRadPerSec() + feederSimPID.calculate(feederSim.getAngularVelocityRadPerSec()));

    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
    }

    @Override
    public void setFeederVelocity(double velocity) {
        feederSimPID.setSetpoint(velocity);
    }

    @Override
    public void configurePID(double p, double i, double d) {
        feederSimPID.setPID(p, i, d);
    }

    @Override
    public void setFeederVoltage(double volts) {
        feederSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
    }
}
