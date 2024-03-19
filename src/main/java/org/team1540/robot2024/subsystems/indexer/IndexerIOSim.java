package org.team1540.robot2024.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.team1540.robot2024.Constants;

import static org.team1540.robot2024.Constants.Indexer.*;

public class IndexerIOSim implements IndexerIO {


    private final DCMotorSim intakeSim = new DCMotorSim(DCMotor.getNEO(1), INTAKE_GEAR_RATIO, INTAKE_MOI);
    private final DCMotorSim feederSim = new DCMotorSim(DCMotor.getNEO(1), FEEDER_GEAR_RATIO, FEEDER_MOI);
    //    private final SimDeviceSim beamBreakSim = new SimDeviceSim("Indexer Beam Break");
    private final PIDController feederSimPID = new PIDController(FEEDER_KP, FEEDER_KI, FEEDER_KD);
    private boolean isClosedLoop = true;
    private double intakeVoltage = 0.0;
    private double feederVoltage = 0.0;
    private double feederSetpointRPS = 0.0;

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        if (isClosedLoop) {
            feederVoltage = MathUtil.clamp(
                    feederSimPID.calculate(feederSim.getAngularVelocityRPM() / 60, feederSetpointRPS),
                    -12.0, 12.0);
        }
        intakeSim.setInputVoltage(intakeVoltage);
        feederSim.setInputVoltage(feederVoltage);
        intakeSim.update(Constants.LOOP_PERIOD_SECS);
        feederSim.update(Constants.LOOP_PERIOD_SECS);
        inputs.intakeCurrentAmps = intakeSim.getCurrentDrawAmps();
        inputs.intakeVoltage = intakeVoltage;
        inputs.intakeVelocityRPS = intakeSim.getAngularVelocityRPM();
        inputs.feederCurrentAmps = feederSim.getCurrentDrawAmps();
        inputs.feederVoltage = feederVoltage;
        inputs.feederVelocityRPS = feederSim.getAngularVelocityRPM();
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void configureFeederPID(double p, double i, double d) {
        feederSimPID.setPID(p, i, d);
    }

    @Override
    public void setFeederVoltage(double volts) {
        isClosedLoop = false;
        feederVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setFeederVelocity(double velocity) {
        isClosedLoop = true;
        feederSimPID.reset();
        feederSetpointRPS = velocity / 60;
    }

}
