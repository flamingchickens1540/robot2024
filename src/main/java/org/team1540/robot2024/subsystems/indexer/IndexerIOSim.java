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
    private boolean isClosedLoop = true;
    private double intakeVoltage = 0.0;
    private double feederVoltage = 0.0;
    private double feederVelocityRPS = 0.0;

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        if (isClosedLoop) {
            feederVoltage = MathUtil.clamp(feederSimPID.calculate(feederSim.getAngularVelocityRPM() / 60, feederVelocityRPS), -12.0, 12.0);
        }
        intakeSim.setInputVoltage(intakeVoltage);
        feederSim.setInputVoltage(feederVoltage);
        intakeSim.update(Constants.LOOP_PERIOD_SECS);
        feederSim.update(Constants.LOOP_PERIOD_SECS);
        inputs.intakeCurrentAmps = intakeSim.getCurrentDrawAmps();
        inputs.intakeVoltage = intakeVoltage;
        inputs.intakeVelocityRPM = intakeSim.getAngularVelocityRPM();
        inputs.feederCurrentAmps = feederSim.getCurrentDrawAmps();
        inputs.feederVoltage = feederVoltage;
        inputs.feederVelocityRPM = feederSim.getAngularVelocityRPM();
//        inputs.noteInIntake = beamBreakSim.getBoolean("Indexer Beam Break").get();
        inputs.setpointRPM = feederSimPID.getSetpoint() * 60;
        inputs.feederPositionError = feederSimPID.getPositionError();
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void configurePID(double p, double i, double d) {
        feederSimPID.setPID(p, i, d);
    }

    public void setFeederVoltage(double volts) {
        isClosedLoop = false;
        feederVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    }


    public void setFeederVelocity(double velocity) {
        isClosedLoop = true;
        feederSimPID.reset();
        feederVelocityRPS = velocity / 60;
    }

}
