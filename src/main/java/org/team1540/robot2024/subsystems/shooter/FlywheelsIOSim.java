package org.team1540.robot2024.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.team1540.robot2024.Constants;

import static org.team1540.robot2024.Constants.Shooter.Flywheels.*;

public class FlywheelsIOSim implements FlywheelsIO{
    private final FlywheelSim leftSim =
            new FlywheelSim(DCMotor.getFalcon500(1), GEAR_RATIO, MOI);
    private final FlywheelSim rightSim =
            new FlywheelSim(DCMotor.getFalcon500(1), GEAR_RATIO, MOI);

    private final PIDController rightController = new PIDController(KP, KI, KD);
    private final PIDController leftController = new PIDController(KP, KI, KD);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV);

    private boolean isClosedLoop;
    private double leftSetpointRPS;
    private double rightSetpointRPS;

    private double leftVolts = 0.0;
    private double rightVolts = 0.0;

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
        if (isClosedLoop) {
            leftVolts =
                    leftController.calculate(leftSim.getAngularVelocityRPM() / 60, leftSetpointRPS)
                    + feedforward.calculate(leftSetpointRPS);
            rightVolts =
                    rightController.calculate(rightSim.getAngularVelocityRPM() / 60, rightSetpointRPS)
                    + feedforward.calculate(rightSetpointRPS);
        }

        leftSim.setInputVoltage(leftVolts);
        rightSim.setInputVoltage(rightVolts);
        leftSim.update(Constants.LOOP_PERIOD_SECS);
        rightSim.update(Constants.LOOP_PERIOD_SECS);

        inputs.leftVelocityRPM = leftSim.getAngularVelocityRPM();
        inputs.leftAppliedVolts = leftVolts;
        inputs.leftCurrentAmps = leftSim.getCurrentDrawAmps();

        inputs.rightVelocityRPM = rightSim.getAngularVelocityRPM();
        inputs.rightAppliedVolts = rightVolts;
        inputs.rightCurrentAmps = rightSim.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        isClosedLoop = false;
        this.leftVolts = leftVolts;
        this.rightVolts = rightVolts;
    }

    @Override
    public void setSpeeds(double leftRPM, double rightRPM) {
        isClosedLoop = true;
        leftController.reset();
        rightController.reset();
        leftSetpointRPS = leftRPM / 60;
        rightSetpointRPS = rightRPM / 60;
    }

    @Override
    public void configPID(double kP, double kI, double kD) {
        leftController.setPID(kP, kI, kD);
        rightController.setPID(kP, kI, kD);
    }
}
