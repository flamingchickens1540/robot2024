package org.team1540.robot2024.subsystems.shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.team1540.robot2024.Constants;

import static org.team1540.robot2024.Constants.Shooter.Pivot.*;

public class ShooterPivotIOSim implements ShooterPivotIO {
    private final SingleJointedArmSim sim =
            new SingleJointedArmSim(
                    DCMotor.getFalcon500Foc(1),
                    TOTAL_GEAR_RATIO,
                    SIM_MOI,
                    SIM_LENGTH_METERS,
                    MIN_ANGLE.getRadians(),
                    MAX_ANGLE.getRadians(),
                    true,
                    MIN_ANGLE.getRadians());

    private final ProfiledPIDController controller =
            new ProfiledPIDController(SIM_KP, SIM_KI, SIM_KD, new TrapezoidProfile.Constraints(CRUISE_VELOCITY_RPS, MAX_ACCEL_RPS2));
    private final ArmFeedforward feedforward = new ArmFeedforward(SIM_KS, SIM_KG, SIM_KV);

    private boolean isClosedLoop;
    private TrapezoidProfile.State goalState;

    private double appliedVolts;

    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {
        if (isClosedLoop) {
            appliedVolts =
                    controller.calculate(Units.radiansToRotations(sim.getAngleRads()), goalState)
                            + feedforward.calculate(
                                    Units.rotationsToRadians(controller.getSetpoint().position),
                                    controller.getSetpoint().velocity);
        }

        sim.setInputVoltage(appliedVolts);
        sim.update(Constants.LOOP_PERIOD_SECS);
        inputs.position = Rotation2d.fromRadians(sim.getAngleRads());
        inputs.absolutePosition = Rotation2d.fromRadians(sim.getAngleRads());
        inputs.velocityRPS = Units.radiansToRotations(sim.getVelocityRadPerSec());
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setPosition(Rotation2d position) {
        controller.reset(
                Units.radiansToRotations(sim.getAngleRads()),
                Units.radiansToRotations(sim.getVelocityRadPerSec())
        );
        isClosedLoop = true;
        goalState = new TrapezoidProfile.State(position.getRotations(), 0);
    }

    @Override
    public void setVoltage(double volts) {
        isClosedLoop = false;
        appliedVolts = volts;
    }

    @Override
    public void configPID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }
}
